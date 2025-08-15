// system.c - Main System Implementation
#include "system.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

// Global system state
SystemState_Global_t g_system_state;
static SystemConfig_t g_system_config;
static uint32_t g_system_start_time;

// External hardware handles
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern Motor_t leftMotor;
extern Motor_t rightMotor;
extern MenuContext_t sysMenu;

// Private function prototypes
static void System_InitializeDefaultConfig(void);
static void System_StateMachine(void);
static void System_UpdateSensors(void);
static void System_UpdateMotors(void);
static void System_UpdateNavigation(void);
static void System_CheckSafety(void);
static bool System_LoadConfigFromMemory(SystemConfig_t *config);
static bool System_SaveConfigToMemory(const SystemConfig_t *config);
static float System_CalculatePIDOutput(float error, float dt, 
                                       float kp, float ki, float kd, 
                                       float *integral, float *prev_error);

// System initialization
bool System_Init(void) {
    // Clear global state
    memset(&g_system_state, 0, sizeof(SystemState_Global_t));
    
    // Initialize default configuration
    System_InitializeDefaultConfig();
    
    // Try to load configuration from memory
    if (!System_LoadConfigFromMemory(&g_system_config)) {
        System_SetDefaultConfig(&g_system_config);
    }
    
    // Initialize starting position and state
    g_system_state.pose.x_mm = 0.0f;
    g_system_state.pose.y_mm = 0.0f;
    g_system_state.pose.heading_rad = 0.0f;
    g_system_state.pose.velocity_mms = 0.0f;
    g_system_state.pose.angular_velocity_rads = 0.0f;
    
    g_system_state.current_state = SYSTEM_STATE_INIT;
    g_system_state.previous_state = SYSTEM_STATE_INIT;
    g_system_state.error_flags = SYSTEM_ERROR_NONE;
    
    g_system_state.sensors_ready = false;
    g_system_state.motors_ready = false;
    g_system_state.navigation_ready = false;
    g_system_state.emergency_stop_active = false;
    
    g_system_start_time = HAL_GetTick();
    g_system_state.state_entry_time_ms = g_system_start_time;
    g_system_state.last_update_time_ms = g_system_start_time;
    
    // Initialize subsystems
    SensorConfig_t sensor_config = {
        .pwm_frequency_hz = 40000,
        .sampling_frequency_hz = 100,
        .wall_threshold_mm = g_system_config.safety.min_wall_distance_mm,
        .alignment_threshold_mm = 10,
        .auto_calibration_enabled = true
    };
    
    if (!Sensor_Init(&sensor_config)) {
        System_SetError(SYSTEM_ERROR_SENSOR_FAILURE);
        return false;
    }
    g_system_state.sensors_ready = true;
    
    // Initialize motor system
    MotorConfig_t motor_config = {
        .control_frequency_hz = SYSTEM_CONTROL_FREQUENCY_HZ,
        .encoder_cpr = SYSTEM_ENCODER_CPR,
        .wheel_diameter_mm = SYSTEM_WHEEL_DIAMETER_MM,
        .gear_ratio = SYSTEM_GEAR_RATIO
    };
    
    MotorSystem_Init(&motor_config);
    
    // Configure motor PID controllers
    Motor_SetPIDGains(&leftMotor, 
                     g_system_config.control_gains.linear_kp,
                     g_system_config.control_gains.linear_ki,
                     g_system_config.control_gains.linear_kd, true);
    
    Motor_SetPIDGains(&rightMotor,
                     g_system_config.control_gains.linear_kp,
                     g_system_config.control_gains.linear_ki,
                     g_system_config.control_gains.linear_kd, true);
    
    g_system_state.motors_ready = true;
    
    // Initialize maze system
    MazeConfig_t maze_config = g_system_config.maze_config;
    Maze_Init(&maze_config);
    g_system_state.navigation_ready = true;
    
    // Set initial state
    System_SetState(SYSTEM_STATE_MENU);
    
    return true;
}

static void System_InitializeDefaultConfig(void) {
    System_SetDefaultConfig(&g_system_config);
}

void System_SetDefaultConfig(SystemConfig_t *config) {
    if (!config) return;
    
    // Motor configuration
    config->motor_config.control_frequency_hz = SYSTEM_CONTROL_FREQUENCY_HZ;
    config->motor_config.encoder_cpr = SYSTEM_ENCODER_CPR;
    config->motor_config.wheel_diameter_mm = SYSTEM_WHEEL_DIAMETER_MM;
    config->motor_config.gear_ratio = SYSTEM_GEAR_RATIO;
    
    // Sensor configuration
    config->sensor_config.pwm_frequency_hz = 40000;
    config->sensor_config.sampling_frequency_hz = 100;
    config->sensor_config.wall_threshold_mm = DEFAULT_MIN_WALL_DIST;
    config->sensor_config.alignment_threshold_mm = 10;
    config->sensor_config.auto_calibration_enabled = true;
    
    // Maze configuration
    config->maze_config.exploration_speed_mms = 200;
    config->maze_config.speed_run_speed_mms = 800;
    config->maze_config.turn_speed_mms = 150;
    config->maze_config.use_diagonal_moves = false;
    config->maze_config.conservative_exploration = true;
    config->maze_config.max_exploration_runs = 3;
    
    // Motion limits
    config->motion_limits.max_linear_velocity_mms = DEFAULT_MAX_LINEAR_VEL;
    config->motion_limits.max_angular_velocity_rads = DEFAULT_MAX_ANGULAR_VEL;
    config->motion_limits.linear_acceleration_mms2 = DEFAULT_LINEAR_ACCEL;
    config->motion_limits.angular_acceleration_rads2 = DEFAULT_ANGULAR_ACCEL;
    
    // Control gains
    config->control_gains.linear_kp = DEFAULT_LINEAR_KP;
    config->control_gains.linear_ki = DEFAULT_LINEAR_KI;
    config->control_gains.linear_kd = DEFAULT_LINEAR_KD;
    config->control_gains.angular_kp = DEFAULT_ANGULAR_KP;
    config->control_gains.angular_ki = DEFAULT_ANGULAR_KI;
    config->control_gains.angular_kd = DEFAULT_ANGULAR_KD;
    config->control_gains.wall_follow_kp = 1.0f;
    config->control_gains.wall_follow_kd = 0.1f;
    
    // Safety parameters
    config->safety.min_wall_distance_mm = DEFAULT_MIN_WALL_DIST;
    config->safety.max_sensor_variance = DEFAULT_MAX_SENSOR_VAR;
    config->safety.watchdog_timeout_ms = DEFAULT_WATCHDOG_TIMEOUT;
}

// Main system update loop
void System_Update(void) {
    uint32_t current_time = HAL_GetTick();
    
    // Update timing
    g_system_state.last_update_time_ms = current_time;
    
    // Safety checks first
    System_CheckSafety();
    
    if (g_system_state.emergency_stop_active) {
        System_SetState(SYSTEM_STATE_ERROR);
        return;
    }
    
    // Update subsystems
    System_UpdateSensors();
    System_UpdateOdometry();
    System_UpdateMotors();
    
    // Run state machine
    System_StateMachine();
    
    // Update statistics
    System_UpdateStats();
    
    // Update display
    System_UpdateDisplay();
}

static void System_StateMachine(void) {
    uint32_t current_time = HAL_GetTick();
    uint32_t state_duration = current_time - g_system_state.state_entry_time_ms;
    
    switch (g_system_state.current_state) {
        case SYSTEM_STATE_INIT:
            // Initialization complete, move to menu
            System_SetState(SYSTEM_STATE_MENU);
            break;
            
        case SYSTEM_STATE_MENU:
            // Menu handling is done by the menu system
            // State changes are triggered by menu actions
            break;
            
        case SYSTEM_STATE_CALIBRATION:
            // Handle calibration timeout
            if (state_duration > 30000) { // 30 second timeout
                System_SetError(SYSTEM_ERROR_CALIBRATION_FAILED);
                System_SetState(SYSTEM_STATE_ERROR);
            }
            break;
            
        case SYSTEM_STATE_EXPLORATION:
            System_UpdateNavigation();
            
            // Check if exploration is complete
            if (Maze_GetState() == MAZE_STATE_COMPLETE) {
                System_SetState(SYSTEM_STATE_SPEED_RUN);
            }
            break;
            
        case SYSTEM_STATE_SPEED_RUN:
            System_UpdateNavigation();
            
            // Check if speed run is complete
            if (Maze_GetState() == MAZE_STATE_COMPLETE) {
                System_SetState(SYSTEM_STATE_IDLE);
            }
            break;
            
        case SYSTEM_STATE_ERROR:
            // Stop all motors
            System_StopRobot();
            
            // Try to recover after 5 seconds if not emergency stop
            if (!g_system_state.emergency_stop_active && state_duration > 5000) {
                // Clear non-critical errors
                if (!(g_system_state.error_flags & (SYSTEM_ERROR_SENSOR_FAILURE | 
                                                   SYSTEM_ERROR_MOTOR_FAILURE))) {
                    g_system_state.error_flags = SYSTEM_ERROR_NONE;
                    System_SetState(SYSTEM_STATE_MENU);
                }
            }
            break;
            
        case SYSTEM_STATE_IDLE:
            // Robot is idle, waiting for user input
            break;
    }
}

static void System_UpdateSensors(void) {
    static uint32_t last_sensor_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Update sensors at 100Hz
    if (current_time - last_sensor_update >= 10) {
        Sensor_StartReading(SENSOR_MODE_SHORT_RANGE);
        last_sensor_update = current_time;
    }
}

void System_UpdateOdometry(void) {
    static int32_t last_left_encoder = 0;
    static int32_t last_right_encoder = 0;
    static uint32_t last_time = 0;
    
    uint32_t current_time = HAL_GetTick();
    float dt = (current_time - last_time) / 1000.0f;
    
    if (dt <= 0.0f) return;
    
    // Read encoder values
    int32_t left_encoder = Motor_GetPosition(&leftMotor);
    int32_t right_encoder = Motor_GetPosition(&rightMotor);
    
    // Calculate encoder deltas
    int32_t left_delta = left_encoder - last_left_encoder;
    int32_t right_delta = right_encoder - last_right_encoder;
    
    // Convert to distances
    float counts_per_mm = SYSTEM_ENCODER_CPR / (M_PI * SYSTEM_WHEEL_DIAMETER_MM);
    float left_distance = left_delta / counts_per_mm;
    float right_distance = right_delta / counts_per_mm;
    
    // Calculate robot motion
    float distance = (left_distance + right_distance) / 2.0f;
    float angle_change = (right_distance - left_distance) / SYSTEM_WHEEL_BASE_MM;
    
    // Update velocities
    g_system_state.pose.velocity_mms = distance / dt;
    g_system_state.pose.angular_velocity_rads = angle_change / dt;
    
    // Update position
    float avg_angle = g_system_state.pose.heading_rad + angle_change / 2.0f;
    g_system_state.pose.x_mm += distance * cosf(avg_angle);
    g_system_state.pose.y_mm += distance * sinf(avg_angle);
    g_system_state.pose.heading_rad += angle_change;
    
    // Normalize heading
    while (g_system_state.pose.heading_rad > M_PI) {
        g_system_state.pose.heading_rad -= 2.0f * M_PI;
    }
    while (g_system_state.pose.heading_rad < -M_PI) {
        g_system_state.pose.heading_rad += 2.0f * M_PI;
    }
    
    // Update previous values
    last_left_encoder = left_encoder;
    last_right_encoder = right_encoder;
    last_time = current_time;
}

static void System_UpdateMotors(void) {
    if (!g_system_state.motors_ready || g_system_state.emergency_stop_active) {
        Motor_SetSpeed(&leftMotor, 0);
        Motor_SetSpeed(&rightMotor, 0);
        return;
    }
    
    // Apply motion command
    if (g_system_state.motion_cmd.emergency_stop) {
        Motor_SetSpeed(&leftMotor, 0);
        Motor_SetSpeed(&rightMotor, 0);
        return;
    }
    
    // Convert motion command to wheel speeds
    float linear_vel = g_system_state.motion_cmd.linear_velocity_mms;
    float angular_vel = g_system_state.motion_cmd.angular_velocity_rads;
    
    // Apply motion limits
    if (linear_vel > g_system_config.motion_limits.max_linear_velocity_mms) {
        linear_vel = g_system_config.motion_limits.max_linear_velocity_mms;
    }
    if (linear_vel < -g_system_config.motion_limits.max_linear_velocity_mms) {
        linear_vel = -g_system_config.motion_limits.max_linear_velocity_mms;
    }
    
    if (angular_vel > g_system_config.motion_limits.max_angular_velocity_rads) {
        angular_vel = g_system_config.motion_limits.max_angular_velocity_rads;
    }
    if (angular_vel < -g_system_config.motion_limits.max_angular_velocity_rads) {
        angular_vel = -g_system_config.motion_limits.max_angular_velocity_rads;
    }
    
    // Calculate wheel speeds
    float wheel_base_half = SYSTEM_WHEEL_BASE_MM / 2.0f;
    float left_speed = linear_vel - angular_vel * wheel_base_half;
    float right_speed = linear_vel + angular_vel * wheel_base_half;
    
    // Convert to PWM values (simplified)
    int32_t left_pwm = (int32_t)(left_speed * 65535.0f / g_system_config.motion_limits.max_linear_velocity_mms);
    int32_t right_pwm = (int32_t)(right_speed * 65535.0f / g_system_config.motion_limits.max_linear_velocity_mms);
    
    // Set motor speeds
    Motor_SetSpeed(&leftMotor, left_pwm);
    Motor_SetSpeed(&rightMotor, right_pwm);
}

static void System_UpdateNavigation(void) {
    if (!g_system_state.navigation_ready) return;
    
    static uint32_t last_nav_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Update navigation at 10Hz
    if (current_time - last_nav_update < 100) return;
    last_nav_update = current_time;
    
    // Get current sensor data for wall detection
    WallDetection_t walls = Sensor_DetectWalls();
    
    // Update maze with wall information (simplified)
    int8_t cell_x = (int8_t)(g_system_state.pose.x_mm / 180.0f);
    int8_t cell_y = (int8_t)(g_system_state.pose.y_mm / 180.0f);
    
    if (walls.left_wall) {
        Maze_SetWall(cell_x, cell_y, DIR_WEST, true);
    }
    if (walls.right_wall) {
        Maze_SetWall(cell_x, cell_y, DIR_EAST, true);
    }
    if (walls.front_wall) {
        Maze_SetWall(cell_x, cell_y, DIR_NORTH, true);
    }
    
    // Get next move from maze solver
    MoveCommand_t next_move = MOVE_STOP;
    
    if (g_system_state.current_state == SYSTEM_STATE_EXPLORATION) {
        next_move = Maze_GetNextExplorationMove();
    } else if (g_system_state.current_state == SYSTEM_STATE_SPEED_RUN) {
        next_move = Maze_GetNextSpeedRunMove();
    }
    
    // Execute move command
    MotionCommand_t motion_cmd = {0};
    
    switch (next_move) {
        case MOVE_FORWARD:
            motion_cmd.linear_velocity_mms = g_system_config.maze_config.exploration_speed_mms;
            motion_cmd.angular_velocity_rads = 0.0f;
            break;
            
        case MOVE_TURN_LEFT:
            motion_cmd.linear_velocity_mms = 0.0f;
            motion_cmd.angular_velocity_rads = g_system_config.motion_limits.max_angular_velocity_rads;
            break;
            
        case MOVE_TURN_RIGHT:
            motion_cmd.linear_velocity_mms = 0.0f;
            motion_cmd.angular_velocity_rads = -g_system_config.motion_limits.max_angular_velocity_rads;
            break;
            
        case MOVE_TURN_AROUND:
            motion_cmd.linear_velocity_mms = 0.0f;
            motion_cmd.angular_velocity_rads = g_system_config.motion_limits.max_angular_velocity_rads;
            break;
            
        case MOVE_STOP:
        default:
            motion_cmd.linear_velocity_mms = 0.0f;
            motion_cmd.angular_velocity_rads = 0.0f;
            break;
    }
    
    System_SetMotionCommand(&motion_cmd);
}

static void System_CheckSafety(void) {
    uint32_t current_time = HAL_GetTick();
    
    // Check sensor status
    if (!Sensor_IsReliable(SENSOR_LEFT) || !Sensor_IsReliable(SENSOR_RIGHT) ||
        !Sensor_IsReliable(SENSOR_FRONT_LEFT) || !Sensor_IsReliable(SENSOR_FRONT_RIGHT)) {
        System_SetError(SYSTEM_ERROR_SENSOR_FAILURE);
    }
    
    // Check for front wall collision
    WallDetection_t walls = Sensor_DetectWalls();
    if (walls.front_wall && walls.front_distance_mm < g_system_config.safety.min_wall_distance_mm) {
        if (g_system_state.pose.velocity_mms > 0.0f) {
            System_EmergencyStop();
        }
    }
    
    // Check watchdog timeout
    if (current_time - g_system_state.last_update_time_ms > g_system_config.safety.watchdog_timeout_ms) {
        System_SetError(SYSTEM_ERROR_TIMEOUT);
    }
}

// State management functions
void System_SetState(SystemState_t new_state) {
    if (!System_IsStateTransitionValid(g_system_state.current_state, new_state)) {
        return;
    }
    
    g_system_state.previous_state = g_system_state.current_state;
    g_system_state.current_state = new_state;
    g_system_state.state_entry_time_ms = HAL_GetTick();
    
    // Call user callback
    System_OnStateChange(g_system_state.previous_state, new_state);
}

SystemState_t System_GetState(void) {
    return g_system_state.current_state;
}

bool System_IsStateTransitionValid(SystemState_t from, SystemState_t to) {
    // Define valid state transitions
    switch (from) {
        case SYSTEM_STATE_INIT:
            return (to == SYSTEM_STATE_MENU || to == SYSTEM_STATE_ERROR);
            
        case SYSTEM_STATE_MENU:
            return (to == SYSTEM_STATE_CALIBRATION || to == SYSTEM_STATE_EXPLORATION ||
                   to == SYSTEM_STATE_SPEED_RUN || to == SYSTEM_STATE_IDLE || to == SYSTEM_STATE_ERROR);
            
        case SYSTEM_STATE_CALIBRATION:
            return (to == SYSTEM_STATE_MENU || to == SYSTEM_STATE_ERROR);
            
        case SYSTEM_STATE_EXPLORATION:
            return (to == SYSTEM_STATE_SPEED_RUN || to == SYSTEM_STATE_MENU || 
                   to == SYSTEM_STATE_IDLE || to == SYSTEM_STATE_ERROR);
            
        case SYSTEM_STATE_SPEED_RUN:
            return (to == SYSTEM_STATE_MENU || to == SYSTEM_STATE_IDLE || to == SYSTEM_STATE_ERROR);
            
        case SYSTEM_STATE_ERROR:
            return (to == SYSTEM_STATE_MENU || to == SYSTEM_STATE_IDLE);
            
        case SYSTEM_STATE_IDLE:
            return (to == SYSTEM_STATE_MENU || to == SYSTEM_STATE_EXPLORATION ||
                   to == SYSTEM_STATE_SPEED_RUN || to == SYSTEM_STATE_ERROR);
    }
    
    return false;
}

// Motion control functions
void System_SetMotionCommand(const MotionCommand_t *cmd) {
    if (!cmd) return;
    
    g_system_state.motion_cmd = *cmd;
}

bool System_MoveForward(float distance_mm, float speed_mms) {
    if (g_system_state.emergency_stop_active) return false;
    
    MotionCommand_t cmd = {
        .linear_velocity_mms = speed_mms,
        .angular_velocity_rads = 0.0f,
        .duration_ms = (uint32_t)(distance_mm / speed_mms * 1000.0f),
        .emergency_stop = false
    };
    
    System_SetMotionCommand(&cmd);
    return true;
}

bool System_TurnLeft(void) {
    if (g_system_state.emergency_stop_active) return false;
    
    MotionCommand_t cmd = {
        .linear_velocity_mms = 0.0f,
        .angular_velocity_rads = g_system_config.motion_limits.max_angular_velocity_rads,
        .duration_ms = (uint32_t)(M_PI_2 / g_system_config.motion_limits.max_angular_velocity_rads * 1000.0f),
        .emergency_stop = false
    };
    
    System_SetMotionCommand(&cmd);
    return true;
}

bool System_TurnRight(void) {
    if (g_system_state.emergency_stop_active) return false;
    
    MotionCommand_t cmd = {
        .linear_velocity_mms = 0.0f,
        .angular_velocity_rads = -g_system_config.motion_limits.max_angular_velocity_rads,
        .duration_ms = (uint32_t)(M_PI_2 / g_system_config.motion_limits.max_angular_velocity_rads * 1000.0f),
        .emergency_stop = false
    };
    
    System_SetMotionCommand(&cmd);
    return true;
}

bool System_TurnAround(void) {
    if (g_system_state.emergency_stop_active) return false;
    
    MotionCommand_t cmd = {
        .linear_velocity_mms = 0.0f,
        .angular_velocity_rads = g_system_config.motion_limits.max_angular_velocity_rads,
        .duration_ms = (uint32_t)(M_PI / g_system_config.motion_limits.max_angular_velocity_rads * 1000.0f),
        .emergency_stop = false
    };
    
    System_SetMotionCommand(&cmd);
    return true;
}

bool System_StopRobot(void) {
    MotionCommand_t cmd = {
        .linear_velocity_mms = 0.0f,
        .angular_velocity_rads = 0.0f,
        .duration_ms = 0,
        .emergency_stop = false
    };
    
    System_SetMotionCommand(&cmd);
    return true;
}

// Wall following functions
float System_GetWallFollowError(void) {
    WallDetection_t walls = Sensor_DetectWalls();
    return walls.alignment_error;
}

void System_AlignWithWalls(void) {
    float error = System_GetWallFollowError();
    
    if (fabsf(error) < 2.0f) return; // Already aligned
    
    float correction = error * g_system_config.control_gains.wall_follow_kp;
    
    MotionCommand_t cmd = {
        .linear_velocity_mms = 0.0f,
        .angular_velocity_rads = correction,
        .duration_ms = 200,
        .emergency_stop = false
    };
    
    System_SetMotionCommand(&cmd);
}

bool System_IsAligned(void) {
    float error = System_GetWallFollowError();
    return fabsf(error) < 2.0f; // 2mm tolerance
}

// Safety functions
void System_EmergencyStop(void) {
    g_system_state.emergency_stop_active = true;
    g_system_state.motion_cmd.emergency_stop = true;
    g_system_state.motion_cmd.linear_velocity_mms = 0.0f;
    g_system_state.motion_cmd.angular_velocity_rads = 0.0f;
    
    Motor_EmergencyStop(&leftMotor);
    Motor_EmergencyStop(&rightMotor);
    
    System_SetError(SYSTEM_ERROR_TIMEOUT);
    System_OnError(SYSTEM_ERROR_TIMEOUT);
}

void System_SetError(SystemError_t error) {
    g_system_state.error_flags |= error;
    System_OnError(error);
}

void System_ClearError(SystemError_t error) {
    g_system_state.error_flags &= ~error;
}

bool System_HasError(SystemError_t error) {
    return (g_system_state.error_flags & error) != 0;
}

const char* System_GetErrorString(SystemError_t error) {
    switch (error) {
        case SYSTEM_ERROR_NONE: return "No Error";
        case SYSTEM_ERROR_SENSOR_FAILURE: return "Sensor Failure";
        case SYSTEM_ERROR_MOTOR_FAILURE: return "Motor Failure";
        case SYSTEM_ERROR_NAVIGATION_ERROR: return "Navigation Error";
        case SYSTEM_ERROR_MEMORY_ERROR: return "Memory Error";
        case SYSTEM_ERROR_TIMEOUT: return "Timeout";
        case SYSTEM_ERROR_CALIBRATION_FAILED: return "Calibration Failed";
        default: return "Unknown Error";
    }
}

// Statistics functions
SystemStats_t System_GetStats(void) {
    return g_system_state.stats;
}

void System_ResetStats(void) {
    memset(&g_system_state.stats, 0, sizeof(SystemStats_t));
}

void System_UpdateStats(void) {
    uint32_t current_time = HAL_GetTick();
    g_system_state.stats.total_runtime_ms = current_time - g_system_start_time;
    
    if (g_system_state.current_state == SYSTEM_STATE_EXPLORATION) {
        static uint32_t exploration_start = 0;
        if (exploration_start == 0) exploration_start = current_time;
        g_system_state.stats.exploration_time_ms = current_time - exploration_start;
    }
    
    if (g_system_state.current_state == SYSTEM_STATE_SPEED_RUN) {
        static uint32_t speed_run_start = 0;
        if (speed_run_start == 0) speed_run_start = current_time;
        g_system_state.stats.speed_run_time_ms = current_time - speed_run_start;
    }
}

// Utility functions
RobotPose_t System_GetPose(void) {
    return g_system_state.pose;
}

uint32_t System_GetUptime(void) {
    return HAL_GetTick() - g_system_start_time;
}

// Display functions
void System_UpdateDisplay(void) {
    // This is called by the main update loop
    // Display updates are handled by the menu system
}

void System_ShowStatus(void) {
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "State: %d", g_system_state.current_state);
    SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY(0, 10);
    snprintf(buffer, sizeof(buffer), "X:%.1f Y:%.1f", 
             g_system_state.pose.x_mm, g_system_state.pose.y_mm);
    SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY(0, 20);
		snprintf(buffer, sizeof(buffer), "Vel:%.1f", g_system_state.pose.velocity_mms);
    SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY(0, 30);
    snprintf(buffer, sizeof(buffer), "Err:0x%02X", g_system_state.error_flags);
    SSD1306_Puts(buffer, &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_UpdateScreen();
}

void System_ShowError(SystemError_t error) {
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("ERROR!", &Font_11x18, SSD1306_COLOR_WHITE);
    
    SSD1306_GotoXY(0, 20);
    const char* error_str = System_GetErrorString(error);
    SSD1306_Puts((char*)error_str, &Font_7x10, SSD1306_COLOR_WHITE);
    
    SSD1306_UpdateScreen();
}

// Calibration functions
bool System_CalibrateMotors(void) {
    if (g_system_state.current_state != SYSTEM_STATE_CALIBRATION) {
        return false;
    }
    
    // Motor calibration sequence
    bool success = true;
    
    // Test left motor
    Motor_SetSpeed(&leftMotor, 32767);  // 50% forward
    HAL_Delay(1000);
    Motor_SetSpeed(&leftMotor, -32767); // 50% reverse
    HAL_Delay(1000);
    Motor_SetSpeed(&leftMotor, 0);      // Stop
    
    // Test right motor
    Motor_SetSpeed(&rightMotor, 32767);  // 50% forward
    HAL_Delay(1000);
    Motor_SetSpeed(&rightMotor, -32767); // 50% reverse
    HAL_Delay(1000);
    Motor_SetSpeed(&rightMotor, 0);      // Stop
    
    // Check encoder functionality
    int32_t left_pos = Motor_GetPosition(&leftMotor);
    int32_t right_pos = Motor_GetPosition(&rightMotor);
    
    if (left_pos == 0 && right_pos == 0) {
        success = false;
        System_SetError(SYSTEM_ERROR_MOTOR_FAILURE);
    }
    
    // Reset encoder positions
    Motor_ResetPosition(&leftMotor);
    Motor_ResetPosition(&rightMotor);
    
    System_OnCalibrationComplete(success);
    return success;
}

bool System_CalibrateSensors(void) {
    if (g_system_state.current_state != SYSTEM_STATE_CALIBRATION) {
        return false;
    }
    
    bool success = Sensor_Calibrate();
    
    if (!success) {
        System_SetError(SYSTEM_ERROR_SENSOR_FAILURE);
    }
    
    System_OnCalibrationComplete(success);
    return success;
}

bool System_CalibrateIMU(void) {
    // IMU calibration placeholder for future implementation
    System_OnCalibrationComplete(true);
    return true;
}

bool System_RunSystemTest(void) {
    bool all_tests_passed = true;
    
    // Test sensors
    if (!Sensor_SelfTest()) {
        System_SetError(SYSTEM_ERROR_SENSOR_FAILURE);
        all_tests_passed = false;
    }
    
    // Test motors
    if (!Motor_SelfTest(&leftMotor) || !Motor_SelfTest(&rightMotor)) {
        System_SetError(SYSTEM_ERROR_MOTOR_FAILURE);
        all_tests_passed = false;
    }
    
    // Test maze system
    if (!Maze_SelfTest()) {
        System_SetError(SYSTEM_ERROR_NAVIGATION_ERROR);
        all_tests_passed = false;
    }
    
    return all_tests_passed;
}

// Configuration management
bool System_LoadConfig(SystemConfig_t *config) {
    return System_LoadConfigFromMemory(config);
}

bool System_SaveConfig(const SystemConfig_t *config) {
    return System_SaveConfigToMemory(config);
}

static bool System_LoadConfigFromMemory(SystemConfig_t *config) {
    // Placeholder for EEPROM/Flash configuration loading
    // In a real implementation, this would read from non-volatile storage
    
    // For now, just return false to use defaults
    return false;
}

static bool System_SaveConfigToMemory(const SystemConfig_t *config) {
    // Placeholder for EEPROM/Flash configuration saving
    // In a real implementation, this would write to non-volatile storage
    
    // For now, just return true to indicate success
    return true;
}

// PID calculation helper
static float System_CalculatePIDOutput(float error, float dt, 
                                       float kp, float ki, float kd, 
                                       float *integral, float *prev_error) {
    if (dt <= 0.0f) return 0.0f;
    
    // Proportional term
    float proportional = kp * error;
    
    // Integral term with windup protection
    *integral += error * dt;
    if (*integral > 1000.0f) *integral = 1000.0f;
    if (*integral < -1000.0f) *integral = -1000.0f;
    float integral_term = ki * (*integral);
    
    // Derivative term
    float derivative = kd * (error - *prev_error) / dt;
    *prev_error = error;
    
    return proportional + integral_term + derivative;
}

// Control loop - called from timer interrupt
void System_ControlLoop(void) {
    static uint32_t control_counter = 0;
    
    control_counter++;
    
    // Run at 1kHz (every call)
    if (g_system_state.sensors_ready) {
        // Update sensor readings if needed
    }
    
    // Run motor control at 1kHz
    if (g_system_state.motors_ready) {
        System_UpdateMotors();
    }
    
    // Run odometry update at 1kHz for best accuracy
    System_UpdateOdometry();
    
    // Run safety checks every 10ms (100Hz)
    if (control_counter % 10 == 0) {
        System_CheckSafety();
    }
}

// Data logging functions (stubs for future implementation)
void System_LogData(const char *message) {
    // Placeholder for data logging
    // Could log to SD card, UART, or other interface
}

void System_LogSensorData(void) {
    // Log current sensor readings
    char buffer[128];
    WallDetection_t walls = Sensor_DetectWalls();
    
    snprintf(buffer, sizeof(buffer), 
             "SENSOR,%.1f,%.1f,%.1f,%.1f,%d,%d,%d\n",
             walls.left_distance_mm,
             walls.right_distance_mm,
             walls.front_distance_mm,
             walls.alignment_error,
             walls.left_wall ? 1 : 0,
             walls.right_wall ? 1 : 0,
             walls.front_wall ? 1 : 0);
    
    System_LogData(buffer);
}

void System_LogMotionData(void) {
    // Log current motion state
    char buffer[128];
    
    snprintf(buffer, sizeof(buffer),
             "MOTION,%.2f,%.2f,%.3f,%.2f,%.3f\n",
             g_system_state.pose.x_mm,
             g_system_state.pose.y_mm,
             g_system_state.pose.heading_rad,
             g_system_state.pose.velocity_mms,
             g_system_state.pose.angular_velocity_rads);
    
    System_LogData(buffer);
}

// System utility functions
uint8_t System_GetCPUUsage(void) {
    // Placeholder for CPU usage calculation
    // Would require implementation of idle time measurement
    return 50; // Return 50% as placeholder
}

uint32_t System_GetFreeRAM(void) {
    // Placeholder for free RAM calculation
    // Would require heap/stack monitoring implementation
    return 8192; // Return 8KB as placeholder
}

float System_GetBatteryVoltage(void) {
    // Placeholder for battery voltage monitoring
    // Would require ADC reading of battery voltage divider
    return 7.4f; // Return 7.4V as placeholder
}

// Weak callback functions - can be overridden by user
__weak void System_OnStateChange(SystemState_t old_state, SystemState_t new_state) {
    // User can override this function for custom state change handling
}

__weak void System_OnError(SystemError_t error) {
    // User can override this function for custom error handling
}

__weak void System_OnMazeComplete(void) {
    // User can override this function for custom maze completion handling
}

__weak void System_OnCalibrationComplete(bool success) {
    // User can override this function for custom calibration completion handling
}
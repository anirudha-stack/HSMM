// system.h - Main System Integration
#ifndef SYSTEM_H
#define SYSTEM_H

#include "stm32f4xx_hal.h"
#include "motor.h"
#include "sensor.h"
#include "maze.h"
#include "menu.h"
#include "ssd1306.h"
#include <stdint.h>
#include <stdbool.h>

// System configuration constants
#define SYSTEM_CONTROL_FREQUENCY_HZ     1000
#define SYSTEM_WHEEL_DIAMETER_MM        32.0f
#define SYSTEM_WHEEL_BASE_MM           90.0f
#define SYSTEM_ENCODER_CPR             1440
#define SYSTEM_GEAR_RATIO              1.0f

// System states
typedef enum {
    SYSTEM_STATE_INIT = 0,
    SYSTEM_STATE_MENU,
    SYSTEM_STATE_CALIBRATION,
    SYSTEM_STATE_EXPLORATION,
    SYSTEM_STATE_SPEED_RUN,
    SYSTEM_STATE_ERROR,
    SYSTEM_STATE_IDLE
} SystemState_t;

// Robot pose structure
typedef struct {
    float x_mm;              // X position in mm
    float y_mm;              // Y position in mm
    float heading_rad;       // Heading in radians
    float velocity_mms;      // Linear velocity in mm/s
    float angular_velocity_rads; // Angular velocity in rad/s
} RobotPose_t;

// Motion commands
typedef struct {
    float linear_velocity_mms;   // Desired linear velocity
    float angular_velocity_rads; // Desired angular velocity
    uint32_t duration_ms;        // Command duration
    bool emergency_stop;         // Emergency stop flag
} MotionCommand_t;

// System statistics
typedef struct {
    uint32_t total_runtime_ms;
    uint32_t exploration_time_ms;
    uint32_t speed_run_time_ms;
    uint16_t cells_explored;
    uint16_t total_distance_mm;
    uint8_t maze_runs_completed;
    uint32_t best_time_ms;
} SystemStats_t;

// Error codes
typedef enum {
    SYSTEM_ERROR_NONE = 0,
    SYSTEM_ERROR_SENSOR_FAILURE = 1,
    SYSTEM_ERROR_MOTOR_FAILURE = 2,
    SYSTEM_ERROR_NAVIGATION_ERROR = 4,
    SYSTEM_ERROR_MEMORY_ERROR = 8,
    SYSTEM_ERROR_TIMEOUT = 16,
    SYSTEM_ERROR_CALIBRATION_FAILED = 32
} SystemError_t;

// System configuration structure
typedef struct {
    // Motor configuration
    MotorConfig_t motor_config;
    
    // Sensor configuration  
    SensorConfig_t sensor_config;
    
    // Maze configuration
    MazeConfig_t maze_config;
    
    // Control parameters
    struct {
        float max_linear_velocity_mms;
        float max_angular_velocity_rads;
        float linear_acceleration_mms2;
        float angular_acceleration_rads2;
    } motion_limits;
    
    // PID parameters for motion control
    struct {
        float linear_kp, linear_ki, linear_kd;
        float angular_kp, angular_ki, angular_kd;
        float wall_follow_kp, wall_follow_kd;
    } control_gains;
    
    // Safety parameters
    struct {
        uint16_t min_wall_distance_mm;
        uint16_t max_sensor_variance;
        uint32_t watchdog_timeout_ms;
    } safety;
    
} SystemConfig_t;

// Global system state
typedef struct {
    SystemState_t current_state;
    SystemState_t previous_state;
    SystemError_t error_flags;
    
    RobotPose_t pose;
    MotionCommand_t motion_cmd;
    SystemStats_t stats;
    
    uint32_t state_entry_time_ms;
    uint32_t last_update_time_ms;
    
    bool emergency_stop_active;
    bool sensors_ready;
    bool motors_ready;
    bool navigation_ready;
    
} SystemState_Global_t;

// Function prototypes

// System initialization and configuration
bool System_Init(void);
bool System_LoadConfig(SystemConfig_t *config);
bool System_SaveConfig(const SystemConfig_t *config);
void System_SetDefaultConfig(SystemConfig_t *config);

// Main system control loop
void System_Update(void);
void System_ControlLoop(void);  // Called from timer interrupt

// State management
void System_SetState(SystemState_t new_state);
SystemState_t System_GetState(void);
bool System_IsStateTransitionValid(SystemState_t from, SystemState_t to);

// Motion control
void System_SetMotionCommand(const MotionCommand_t *cmd);
void System_UpdateOdometry(void);
void System_UpdateMotionControl(void);
RobotPose_t System_GetPose(void);

// Navigation functions
bool System_MoveForward(float distance_mm, float speed_mms);
bool System_TurnLeft(void);
bool System_TurnRight(void);
bool System_TurnAround(void);
bool System_StopRobot(void);

// Wall following and alignment
float System_GetWallFollowError(void);
void System_AlignWithWalls(void);
bool System_IsAligned(void);

// Safety and error handling
void System_EmergencyStop(void);
void System_SetError(SystemError_t error);
void System_ClearError(SystemError_t error);
bool System_HasError(SystemError_t error);
const char* System_GetErrorString(SystemError_t error);

// Calibration functions
bool System_CalibrateMotors(void);
bool System_CalibrateSensors(void);
bool System_CalibrateIMU(void);  // If IMU is added later
bool System_RunSystemTest(void);

// Statistics and monitoring
SystemStats_t System_GetStats(void);
void System_ResetStats(void);
void System_UpdateStats(void);

// Utility functions
uint32_t System_GetUptime(void);
float System_GetBatteryVoltage(void);  // If voltage monitoring is added
uint8_t System_GetCPUUsage(void);
uint32_t System_GetFreeRAM(void);

// Display and user interface
void System_UpdateDisplay(void);
void System_ShowStatus(void);
void System_ShowError(SystemError_t error);

// Data logging (if implemented)
void System_LogData(const char *message);
void System_LogSensorData(void);
void System_LogMotionData(void);

// External hardware interfaces
extern Motor_t leftMotor;
extern Motor_t rightMotor;
extern MenuContext_t sysMenu;

// System callbacks (can be implemented by user)
void System_OnStateChange(SystemState_t old_state, SystemState_t new_state);
void System_OnError(SystemError_t error);
void System_OnMazeComplete(void);
void System_OnCalibrationComplete(bool success);

// Inline utility functions
static inline bool System_IsRunning(void) {
    SystemState_t state = System_GetState();
    return (state != SYSTEM_STATE_ERROR && state != SYSTEM_STATE_INIT);
}

static inline bool System_IsMoving(void) {
    extern SystemState_Global_t g_system_state;
    return (g_system_state.motion_cmd.linear_velocity_mms != 0.0f || 
            g_system_state.motion_cmd.angular_velocity_rads != 0.0f);
}

static inline float System_GetLinearVelocity(void) {
    extern SystemState_Global_t g_system_state;
    return g_system_state.pose.velocity_mms;
}

static inline float System_GetAngularVelocity(void) {
    extern SystemState_Global_t g_system_state;
    return g_system_state.pose.angular_velocity_rads;
}

// Configuration macros
#define SYSTEM_CONFIG_VERSION   1
#define SYSTEM_CONFIG_MAGIC     0xDEADBEEF

// Default configuration values
#define DEFAULT_MAX_LINEAR_VEL      500.0f  // mm/s
#define DEFAULT_MAX_ANGULAR_VEL     3.14f   // rad/s (180 deg/s)
#define DEFAULT_LINEAR_ACCEL        1000.0f // mm/s²
#define DEFAULT_ANGULAR_ACCEL       6.28f   // rad/s²

// PID defaults
#define DEFAULT_LINEAR_KP           2.0f
#define DEFAULT_LINEAR_KI           0.1f
#define DEFAULT_LINEAR_KD           0.05f
#define DEFAULT_ANGULAR_KP          1.5f
#define DEFAULT_ANGULAR_KI          0.05f
#define DEFAULT_ANGULAR_KD          0.02f

// Safety defaults
#define DEFAULT_MIN_WALL_DIST       30      // mm
#define DEFAULT_MAX_SENSOR_VAR      100     // ADC counts
#define DEFAULT_WATCHDOG_TIMEOUT    5000    // ms

#endif // SYSTEM_H
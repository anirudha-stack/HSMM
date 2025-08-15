
// motor.c - Implementation
#include "motor.h"
#include <math.h>
#include <string.h>

static MotorConfig_t motor_config;
static uint32_t system_time_ms = 0;

void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel,
                GPIO_TypeDef *dir_port, uint16_t dir_pin, TIM_HandleTypeDef *htim_encoder) {
    
    // Clear the motor structure
    memset(motor, 0, sizeof(Motor_t));
    
    // Hardware configuration
    motor->htim_pwm = htim_pwm;
    motor->pwm_channel = pwm_channel;
    motor->dir_port = dir_port;
    motor->dir_pin = dir_pin;
    motor->htim_encoder = htim_encoder;
    motor->inverted = false;
    
    // Initialize state
    motor->current_speed = 0;
    motor->target_speed = 0;
    motor->direction = MOTOR_DIR_FORWARD;
    motor->state = MOTOR_STATE_IDLE;
    
    // Default PID parameters (should be tuned for specific application)
    Motor_SetPIDGains(motor, 1.0f, 0.1f, 0.05f, true);   // Speed controller
    Motor_SetPIDGains(motor, 2.0f, 0.0f, 0.1f, false);   // Position controller
    
    // Safety limits
    motor->max_speed_limit = MOTOR_MAX_SPEED * 0.8f;  // 80% of max
    motor->max_acceleration = 5000;  // PWM units per second
    motor->emergency_stop = false;
    
    // Motion profile defaults
    motor->motion_profile.max_velocity = motor->max_speed_limit;
    motor->motion_profile.acceleration = motor->max_acceleration;
    motor->motion_profile.deceleration = motor->max_acceleration;
}

void Motor_SetPIDGains(Motor_t *motor, float kp, float ki, float kd, bool is_speed_controller) {
    PIDController_t *pid = is_speed_controller ? &motor->speed_pid : &motor->position_pid;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_output = MOTOR_MAX_SPEED;
    pid->min_output = -MOTOR_MAX_SPEED;
}

void Motor_StartPWM(Motor_t *motor) {
    HAL_TIM_PWM_Start(motor->htim_pwm, motor->pwm_channel);
    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, 0);
}

static float PID_Update(PIDController_t *pid, float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with windup protection
    pid->integral += error * dt;
    float max_integral = pid->max_output / (pid->ki + 1e-6f);
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    float i_term = pid->ki * pid->integral;
    
    // Derivative term
    float d_term = 0.0f;
    if (dt > 0.0f) {
        d_term = pid->kd * (error - pid->prev_error) / dt;
    }
    pid->prev_error = error;
    
    // Calculate output with limits
    float output = p_term + i_term + d_term;
    if (output > pid->max_output) output = pid->max_output;
    if (output < pid->min_output) output = pid->min_output;
    
    return output;
}

static void Motor_SetPWMAndDirection(Motor_t *motor, int32_t speed) {
    uint32_t pwm_value;
    GPIO_PinState dir_state;
    
    if (speed >= 0) {
        pwm_value = (uint32_t)speed;
        dir_state = motor->inverted ? GPIO_PIN_SET : GPIO_PIN_RESET;
        motor->direction = MOTOR_DIR_FORWARD;
    } else {
        pwm_value = (uint32_t)(-speed);
        dir_state = motor->inverted ? GPIO_PIN_RESET : GPIO_PIN_SET;
        motor->direction = MOTOR_DIR_REVERSE;
    }
    
    // Clamp PWM value
    if (pwm_value > MOTOR_MAX_SPEED) pwm_value = MOTOR_MAX_SPEED;
    
    // Set hardware
    HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, dir_state);
    __HAL_TIM_SET_COMPARE(motor->htim_pwm, motor->pwm_channel, pwm_value);
    
    motor->current_speed = speed >= 0 ? (int32_t)pwm_value : -(int32_t)pwm_value;
}

void Motor_SetSpeed(Motor_t *motor, int32_t speed) {
    if (motor->emergency_stop) return;
    
    // Clamp speed to limits
    if (speed > (int32_t)motor->max_speed_limit) speed = (int32_t)motor->max_speed_limit;
    if (speed < -(int32_t)motor->max_speed_limit) speed = -(int32_t)motor->max_speed_limit;
    
    motor->target_speed = speed;
    
    // For direct speed control, update immediately
    Motor_SetPWMAndDirection(motor, speed);
    motor->state = (speed == 0) ? MOTOR_STATE_IDLE : MOTOR_STATE_CONSTANT_SPEED;
}

void Motor_UpdateControl(Motor_t *motor) {
    if (motor->emergency_stop) return;
    
    motor->control_loop_count++;
    uint32_t current_time = system_time_ms;
    float dt = 0.001f * motor_config.control_frequency_hz; // Assuming 1kHz control
    
    // Update encoder readings
    if (motor->htim_encoder) {
        int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(motor->htim_encoder);
        motor->encoder_velocity = current_count - motor->prev_encoder_count;
        motor->prev_encoder_count = current_count;
        motor->encoder_count = current_count;
    }
    
    // Motion profile implementation would go here
    // For now, use simple speed control
    
    // Speed control loop (if using encoder feedback)
    if (motor->htim_encoder && motor->target_speed != 0) {
        float target_velocity = (float)motor->target_speed / MOTOR_MAX_SPEED * motor->motion_profile.max_velocity;
        float current_velocity = (float)motor->encoder_velocity;
        
        float speed_output = PID_Update(&motor->speed_pid, target_velocity, current_velocity, dt);
        Motor_SetPWMAndDirection(motor, (int32_t)speed_output);
    }
    
    // Update statistics
    motor->average_speed = (motor->average_speed * 0.99f) + (abs(motor->current_speed) * 0.01f);
}

void Motor_EmergencyStop(Motor_t *motor) {
    motor->emergency_stop = true;
    motor->target_speed = 0;
    motor->current_speed = 0;
    motor->state = MOTOR_STATE_ERROR;
    Motor_SetPWMAndDirection(motor, 0);
}

void Motor_Reset(Motor_t *motor) {
    motor->emergency_stop = false;
    motor->target_speed = 0;
    motor->current_speed = 0;
    motor->state = MOTOR_STATE_IDLE;
    motor->speed_pid.integral = 0.0f;
    motor->position_pid.integral = 0.0f;
    motor->error_count = 0;
    Motor_SetPWMAndDirection(motor, 0);
}

// Utility functions
int32_t Motor_GetPosition(Motor_t *motor) {
    return motor->encoder_count;
}

int32_t Motor_GetVelocity(Motor_t *motor) {
    return motor->encoder_velocity;
}

float Motor_GetDistanceMM(Motor_t *motor) {
    float counts_per_mm = motor_config.encoder_cpr / 
                         (M_PI * motor_config.wheel_diameter_mm / motor_config.gear_ratio);
    return (float)motor->encoder_count / counts_per_mm;
}

float Motor_GetSpeedMMS(Motor_t *motor) {
    float counts_per_mm = motor_config.encoder_cpr / 
                         (M_PI * motor_config.wheel_diameter_mm / motor_config.gear_ratio);
    return (float)motor->encoder_velocity * motor_config.control_frequency_hz / counts_per_mm;
}

bool Motor_IsAtTarget(Motor_t *motor) {
    return abs(motor->current_speed - motor->target_speed) <= MOTOR_POSITION_TOLERANCE;
}

MotorState_t Motor_GetState(Motor_t *motor) {
    return motor->state;
}

void MotorSystem_Init(MotorConfig_t *config) {
    motor_config = *config;
}

void MotorSystem_ControlLoop(void) {
    system_time_ms++;
    // This would be called from a timer interrupt
    // Update all motors in the system
}

void MotorSystem_EmergencyStop(void) {
    // Stop all motors in the system
}
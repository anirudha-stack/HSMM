// motor.h - Enhanced Motor Control System
#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Motor configuration constants
#define MOTOR_MAX_SPEED         65535
#define MOTOR_MIN_SPEED         0
#define MOTOR_RAMP_STEP         1000
#define MOTOR_POSITION_TOLERANCE 5

// Motor states for finite state machine
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_ACCELERATING,
    MOTOR_STATE_CONSTANT_SPEED,
    MOTOR_STATE_DECELERATING,
    MOTOR_STATE_BRAKING,
    MOTOR_STATE_ERROR
} MotorState_t;

// Motor direction enumeration
typedef enum {
    MOTOR_DIR_FORWARD = 0,
    MOTOR_DIR_REVERSE = 1
} MotorDirection_t;

// PID controller structure
typedef struct {
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    float integral;     // Integral accumulator
    float prev_error;   // Previous error for derivative
    float max_output;   // Maximum output limit
    float min_output;   // Minimum output limit
    uint32_t last_time; // Last update time (ms)
} PIDController_t;

// Motion profile structure
typedef struct {
    uint32_t target_position;
    uint32_t current_position;
    uint32_t max_velocity;
    uint32_t acceleration;
    uint32_t deceleration;
    MotorState_t state;
} MotionProfile_t;

// Enhanced Motor structure
typedef struct {
    // Hardware configuration
    TIM_HandleTypeDef *htim_pwm;
    uint32_t pwm_channel;
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    TIM_HandleTypeDef *htim_encoder;
    bool inverted;
    
    // Current state
    int32_t current_speed;          // Current PWM duty cycle
    int32_t target_speed;           // Target PWM duty cycle
    MotorDirection_t direction;
    MotorState_t state;
    
    // Encoder data
    volatile int32_t encoder_count;
    volatile int32_t prev_encoder_count;
    volatile int32_t encoder_velocity;  // Counts per control loop
    
    // Control systems
    PIDController_t speed_pid;
    PIDController_t position_pid;
    MotionProfile_t motion_profile;
    
    // Safety and limits
    uint32_t max_speed_limit;
    uint32_t max_acceleration;
    bool emergency_stop;
    
    // Statistics and debugging
    uint32_t control_loop_count;
    uint32_t error_count;
    float average_speed;
} Motor_t;

// Motor system configuration
typedef struct {
    uint32_t control_frequency_hz;  // Control loop frequency
    uint32_t encoder_cpr;          // Counts per revolution
    float wheel_diameter_mm;       // Wheel diameter in mm
    float gear_ratio;              // Gear reduction ratio
} MotorConfig_t;

// Function prototypes
void Motor_Init(Motor_t *motor, TIM_HandleTypeDef *htim_pwm, uint32_t pwm_channel,
                GPIO_TypeDef *dir_port, uint16_t dir_pin, TIM_HandleTypeDef *htim_encoder);
void Motor_SetPIDGains(Motor_t *motor, float kp, float ki, float kd, bool is_speed_controller);
void Motor_StartPWM(Motor_t *motor);
void Motor_SetSpeed(Motor_t *motor, int32_t speed);
void Motor_SetTargetPosition(Motor_t *motor, int32_t position);
void Motor_UpdateControl(Motor_t *motor);
void Motor_EmergencyStop(Motor_t *motor);
void Motor_Reset(Motor_t *motor);

// Utility functions
int32_t Motor_GetPosition(Motor_t *motor);
int32_t Motor_GetVelocity(Motor_t *motor);
float Motor_GetDistanceMM(Motor_t *motor);
float Motor_GetSpeedMMS(Motor_t *motor);
bool Motor_IsAtTarget(Motor_t *motor);
MotorState_t Motor_GetState(Motor_t *motor);

// System-wide motor control
void MotorSystem_Init(MotorConfig_t *config);
void MotorSystem_ControlLoop(void);  // Call from timer interrupt
void MotorSystem_EmergencyStop(void);

#endif // MOTOR_H

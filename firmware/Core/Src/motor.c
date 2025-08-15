#include "motor.h"
#include "main.h"
void Motor_Init(Motor *motor, TIM_HandleTypeDef *htim, uint32_t pwm_channel, GPIO_TypeDef *DIR_GPIO_Port, uint16_t DIR_Pin)
{
    // Initialize encoder and state variables
    motor->encoder_count      = 0;
    motor->prev_encoder_count = 0;
    motor->prev_velocity      = 0;
    motor->prev_direction     = 0;
    motor->velocity           = 0;
    motor->direction          = 0;
    
    // Store hardware handles and channel info
    motor->htim          = htim;
    motor->pwm_channel   = pwm_channel;
    motor->DIR_GPIO_Port = DIR_GPIO_Port;
    motor->DIR_Pin       = DIR_Pin;
}

void Motor_StartPWM(Motor *motor)
{
    // Start PWM for the motor using its stored timer handle and channel.
	  //
    HAL_TIM_PWM_Start(motor->htim, motor->pwm_channel);
	  
}

// helper to (re)configure & start PWM on any timer/channel
static void pwm_set_duty_cycle(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse) {
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = pulse;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(htim, channel);
}

void set_motor_speed(Motor *motor, int32_t speed)
{
    // clamp speed to [-65535, +65535]
    if (speed >  65535) speed =  65535;
    if (speed < -65535) speed = -65535;

    // save previous state
    motor->prev_direction = motor->direction;
    motor->prev_velocity  = motor->velocity;

    // determine base_direction (0 = forward, 1 = reverse)
    uint8_t base_direction = (speed >= 0) ? 0 : 1;
    if (speed < 0) speed = -speed;  // make speed positive for duty

    // apply inversion flag
    uint8_t direction = motor->inverted ? !base_direction : base_direction;
    HAL_GPIO_WritePin(motor->DIR_GPIO_Port, motor->DIR_Pin, (GPIO_PinState)direction);

    // compute dutyCycle from 0–65535 into [0..Period]
    uint32_t period     = motor->htim->Init.Period;
    uint32_t dutyCycle  = ((uint32_t)speed * period) / 65535;

    // update PWM output
    pwm_set_duty_cycle(motor->htim, motor->pwm_channel, dutyCycle);

    // store new state
    motor->direction = direction;
    motor->velocity  = speed;
}




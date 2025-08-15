#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f4xx_hal.h"   // Change this include to match your MCU series
#include <stdint.h>

/* Motor structure definition.
   Members:
    - encoder_count: current encoder reading (update this externally)
    - prev_encoder_count: previous encoder reading (useful for velocity calculation)
    - prev_velocity: previous velocity (or speed) value
    - prev_direction: last direction used (0: forward, 1: reverse)
    - velocity: current absolute speed (0-255)
    - direction: current motor direction (0: forward, 1: reverse)
    - htim: pointer to the timer handle used for PWM generation
    - pwm_channel: the timer channel (e.g., TIM_CHANNEL_1 or TIM_CHANNEL_2)
    - DIR_GPIO_Port & DIR_Pin: GPIO port and pin used to control motor direction
		
		sample usage:
			for (int32_t i = -65535; i < 65535; i += 100) {
    set_motor_speed(&leftMotor, i);
    set_motor_speed(&rightMotor, i);
    HAL_Delay(10); // motor gets time to act on this duty cycle
}
		
		
*/
typedef struct {
    uint32_t encoder_count;
    uint32_t prev_encoder_count;
    uint32_t  prev_velocity;
    uint8_t  prev_direction;
    uint32_t  velocity;      // current absolute speed (0 to 255)
    uint8_t  direction;     // current direction: 0 for forward, 1 for reverse
	  uint8_t  inverted;      // new field: 0 = normal, 1 = inverted logic for direction

    TIM_HandleTypeDef *htim;
    uint32_t pwm_channel;
    GPIO_TypeDef *DIR_GPIO_Port;
    uint16_t DIR_Pin;
} Motor;

/* Initializes a Motor structure.
   @param motor: pointer to the Motor structure to initialize.
   @param htim: pointer to the timer handle for PWM.
   @param pwm_channel: timer PWM channel to use (e.g., TIM_CHANNEL_1 for left motor).
   @param DIR_GPIO_Port: GPIO port for the motor's direction control.
   @param DIR_Pin: GPIO pin for the motor's direction control.
*/
void Motor_Init(Motor *motor, TIM_HandleTypeDef *htim, uint32_t pwm_channel, GPIO_TypeDef *DIR_GPIO_Port, uint16_t DIR_Pin);

/* Starts the motor PWM timer using the configured timer handle and channel.
   @param motor: pointer to the Motor structure.
*/
void Motor_StartPWM(Motor *motor);

/* Sets the motor speed.
   @param motor: pointer to the Motor structure.
   @param speed: speed value from -255 to 255.
                 If speed > 255, it is clamped to 255.
                 If speed < -255, it is clamped to -255.
                 A positive value sets the motor to move forward,
                 a negative value sets it to move backward.
*/
void set_motor_speed(Motor *motor, int32_t speed);

#endif /* MOTOR_H */

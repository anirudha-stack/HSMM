#ifndef INIT_H
#define INIT_H

#include "stm32f4xx_hal.h"  // Adjust this include for your MCU series
#include "motor.h"          // Include any other libraries you want to initialize
#include "ssd1306.h"
#include "sensor.h"
#include "menu.h"
// Extern declarations for global motor instances if you want to use them across your project.
extern Motor leftMotor;
extern Motor rightMotor;



/**
  * @brief  Initializes all libraries and peripherals.
  *         Add any additional library initializations in this function.
  */

void INIT_All(void);

#endif /* INIT_H */

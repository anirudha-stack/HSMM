#include "gpio.h"
#include "init.h"

// Declare motor instances (global variables)
Motor leftMotor;
Motor rightMotor;


MenuContext sysMenu;

// Optional calibration submenu
MenuItem CalibMenu[] = {
    { "Sensor L", ACTION_CALIB_SENSOR, NULL, 0 },
    { "Sensor LF", ACTION_CALIB_SENSOR, NULL, 0 },
		{ "Sensor RF", ACTION_CALIB_SENSOR, NULL, 0 },
    { "Sensor R", ACTION_CALIB_SENSOR, NULL, 0 },
};

MenuItem BootMenu[] = {
    { "Search Maze",      ACTION_SEARCH_MAZE, NULL, 0 },
    { "Run Maze",         ACTION_RUN_MAZE, NULL, 0 },
    { "Calibrate Sensors",ACTION_NONE, CalibMenu, 4 },
    { "Set Max Speeds/PID", ACTION_SET_PID, NULL, 0 },
    { "Test Utility",     ACTION_UTIL_TEST, NULL, 0 },
};


// Extern timer handle and any GPIO definitions are assumed to be defined in other files,
// such as in your CubeMX-generated main.c or main.h.
extern TIM_HandleTypeDef htim1;  // Timer used for motor PWM
extern TIM_HandleTypeDef htim2;  // Timer used for motor PWM
extern TIM_HandleTypeDef htim5;  // Timer used for motor PWM

extern TIM_HandleTypeDef htim4;  // Timer used for motor PWM

// The GPIO ports and pins for motor directions should be defined elsewhere,
// for example, in main.h or as defined by your hardware configuration:
 
 
void INIT_All(void)
{
		SSD1306_Init();
		HAL_GPIO_WritePin(USER_LED_GPIO_Port,USER_LED_Pin,1);
	  SSD1306_Clear();
	  SSD1306_GotoXY(5,5);
		SSD1306_Puts("Woke up...",&Font_7x10,SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		HAL_Delay(550);
    // Initialize the motor library for both motors.
    Motor_Init(&leftMotor,  &htim4, TIM_CHANNEL_4, LEFT_MOT_DIR_GPIO_Port, LEFT_MOT_DIR_Pin);
	  leftMotor.inverted = 0;
    Motor_Init(&rightMotor, &htim1, TIM_CHANNEL_4, RIGHT_MOT_DIR_GPIO_Port, RIGHT_MOT_DIR_Pin);
    rightMotor.inverted = 1;
    // Start PWM for both motors.
    Motor_StartPWM(&leftMotor); 
    Motor_StartPWM(&rightMotor);
		SSD1306_Clear();
	  SSD1306_GotoXY(5,5);
		SSD1306_Puts("MOT READY",&Font_7x10,SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		HAL_Delay(550);

    // Add additional library initializations here if needed.
    // For example, sensor initializations, communication interfaces, etc.
	
	

	
	
	 
	
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
		
		SSD1306_Clear();
	  SSD1306_GotoXY(5,5);
		SSD1306_Puts("ENC READY",&Font_7x10,SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		HAL_Delay(550);
		
		
		 Sensor_Init();
		 
		SSD1306_Clear();
	  SSD1306_GotoXY(5,5);
		SSD1306_Puts("SENSE READY",&Font_7x10,SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
		HAL_Delay(550);
		
		

    Menu_Init(&sysMenu, BootMenu, sizeof(BootMenu)/sizeof(BootMenu[0]));
		
		

}


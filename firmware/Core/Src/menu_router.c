#include "menu_router.h"
#include "ssd1306.h"
#include "fonts.h"
#include "sensor.h"
#include "main.h"
#include <stdio.h>

extern 	SensorRawData sensorData;
extern 	SensorDistData sensorDistData;


// Placeholders for real handlers
void Maze_Search() {
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Maze Search", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}
void Maze_Run() {
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Maze Run", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}
void Calibarate_Sensors() {
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Calib A", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}
void SetPID() {
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Set PID", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
}
void TestUtility() {
    SSD1306_Clear();
    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("Sensor Test...", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
		HAL_Delay(800);
		SSD1306_Clear();
		
	
		while(1){
			Sensor_GetDistance(SHORT_RANGE, &sensorData , &sensorDistData);
			SSD1306_Clear();
			SSD1306_GotoXY(0,0);
			char buf[40];
			snprintf(buf,sizeof(buf),"L:%lu  LF:%lu", sensorDistData.left_mm, sensorDistData.frontLeft_mm);
			SSD1306_Puts(buf,&Font_7x10,SSD1306_COLOR_WHITE);
			SSD1306_GotoXY(0,16);
			snprintf(buf,sizeof(buf),"RF:%lu  R:%lu", sensorDistData.frontRight_mm, sensorDistData.right_mm);
			SSD1306_Puts(buf,&Font_7x10,SSD1306_COLOR_WHITE);
			SSD1306_UpdateScreen();
			
			if(HAL_GPIO_ReadPin(USER_BUTT_GPIO_Port,USER_BUTT_Pin) == 0)
				{
					break;
		    }
			}
		 
	  
	
}

void Menu_ExecuteAction(MenuActionID id) {
    switch (id) {
        case ACTION_SEARCH_MAZE: Maze_Search(); break;
        case ACTION_RUN_MAZE: Maze_Run(); break;
        case ACTION_CALIB_SENSOR:  Calibarate_Sensors(); break;
        case ACTION_SET_PID: SetPID(); break;
        case ACTION_UTIL_TEST: TestUtility(); break;
        default:
            SSD1306_Clear();
            SSD1306_GotoXY(0, 0);
            SSD1306_Puts("No action", &Font_7x10, SSD1306_COLOR_WHITE);
            SSD1306_UpdateScreen();
            break;
    }
}
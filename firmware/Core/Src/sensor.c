// sensor.c
#include "sensor.h"
#include "ssd1306.h"
#include "sensorCalib.h"
#include "stm32f4xx_hal.h"   // for __HAL_TIM_*, HAL_ADC_*

// extern handles from CubeMX
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

// match your TIM3 ARR
#define PWM_FULL  (__HAL_TIM_GET_AUTORELOAD(&htim3))
#define PWM_SHORT (PWM_FULL/6)

void Sensor_Init(void)
{
    // start PWM channels so we can change CCR on the fly
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    // ensure all off
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}

void Sensor_GetReadings(SensorRange range, SensorRawData *out)
{
    uint32_t convVals[4];
    uint32_t duty = (range==FULL_RANGE ? PWM_FULL : PWM_SHORT);

    for (uint8_t em=0; em<4; em++)
    {
        // 1) light only emitter 'em'
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (em==0)? duty:0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (em==1)? duty:0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (em==2)? duty:0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (em==3)? duty:0);
        // force update
        htim3.Instance->EGR = TIM_EGR_UG;

        // 2) start the 4-channel scan
        HAL_ADC_Start(&hadc1);

        // 3) pull each conversion in order
        for (uint8_t rank=0; rank<4; rank++)
        {
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            convVals[rank] = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);

        // 4) pick out only the sensor that matches this emitter
        switch(em) {
            case 0: out->leftSensor = convVals[0]; break;
            case 1: out->frontLeftSensor = convVals[1]; break;
            case 2: out->frontRightSensor = convVals[2]; break;
            default: out->rightSensor = convVals[3]; break;
        }

        // 5) turn this emitter off
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
        htim3.Instance->EGR = TIM_EGR_UG;
    }   
}


void Sensor_GetDistance(
    SensorRange range,
    SensorRawData *outRaw,
    SensorDistData *outDist
) {
    // 1) Acquire raw ADC readings
    Sensor_GetReadings(range, outRaw);

    // 2) Convert each to millimetres using the single generic function
    outDist->left_mm       = ConvertADCtoMM(SENSOR_LEFT,        (uint16_t)outRaw->leftSensor);
    outDist->frontLeft_mm  = ConvertADCtoMM(SENSOR_FRONT_LEFT,  (uint16_t)outRaw->frontLeftSensor);
    outDist->frontRight_mm = ConvertADCtoMM(SENSOR_FRONT_RIGHT, (uint16_t)outRaw->frontRightSensor);
    outDist->right_mm      = ConvertADCtoMM(SENSOR_RIGHT,       (uint16_t)outRaw->rightSensor);
}
// sensor.h
#ifndef __SENSOR_H
#define __SENSOR_H

#include <stdint.h>

typedef enum {
    SHORT_RANGE = 0,
    FULL_RANGE  = 1
} SensorRange;

typedef struct {
    uint32_t leftSensor;
    uint32_t frontLeftSensor;
    uint32_t frontRightSensor;
    uint32_t rightSensor;
} SensorRawData;

typedef struct {
    uint16_t left_mm;
    uint16_t frontLeft_mm;
    uint16_t frontRight_mm;
    uint16_t right_mm;
} SensorDistData;

// call once after MX_*_Init()
void Sensor_Init(void);

// blocking: reads all four emitters+ADC pairs
void Sensor_GetReadings(SensorRange range, SensorRawData *out);
void Sensor_GetDistance(
    SensorRange range,
    SensorRawData *outRaw,
    SensorDistData *outDist
);

#endif // __SENSOR_H

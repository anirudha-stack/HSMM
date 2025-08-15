// sensor.h - Enhanced Sensor Management System
#ifndef SENSOR_H
#define SENSOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// Sensor configuration
#define SENSOR_COUNT                4
#define SENSOR_SAMPLES_PER_READING  8
#define SENSOR_FILTER_DEPTH         4
#define SENSOR_CALIBRATION_POINTS   10
#define SENSOR_MAX_DISTANCE_MM      200
#define SENSOR_MIN_DISTANCE_MM      5

// Sensor identifiers
typedef enum {
    SENSOR_LEFT = 0,
    SENSOR_FRONT_LEFT,
    SENSOR_FRONT_RIGHT,
    SENSOR_RIGHT
} SensorID_t;

// Sensor operating modes
typedef enum {
    SENSOR_MODE_IDLE = 0,
    SENSOR_MODE_SHORT_RANGE,
    SENSOR_MODE_LONG_RANGE,
    SENSOR_MODE_CALIBRATION,
    SENSOR_MODE_ERROR
} SensorMode_t;

// Sensor status flags
typedef enum {
    SENSOR_STATUS_OK = 0,
    SENSOR_STATUS_OUT_OF_RANGE = 1,
    SENSOR_STATUS_UNRELIABLE = 2,
    SENSOR_STATUS_CALIBRATION_NEEDED = 4,
    SENSOR_STATUS_ERROR = 8
} SensorStatus_t;

// Calibration data structure
typedef struct {
    uint16_t adc_values[SENSOR_CALIBRATION_POINTS];
    uint16_t distances_mm[SENSOR_CALIBRATION_POINTS];
    uint8_t point_count;
    bool is_calibrated;
    float polynomial_coeffs[4];  // For polynomial curve fitting
} SensorCalibration_t;

// Moving average filter
typedef struct {
    uint16_t buffer[SENSOR_FILTER_DEPTH];
    uint8_t index;
    uint32_t sum;
    bool initialized;
} MovingAverageFilter_t;

// Individual sensor data
typedef struct {
    SensorID_t id;
    uint8_t adc_channel;
    uint8_t emitter_channel;
    
    // Current readings
    uint16_t raw_adc;
    uint16_t filtered_adc;
    uint16_t distance_mm;
    
    // Status and diagnostics
    SensorStatus_t status;
    SensorMode_t mode;
    uint32_t reading_count;
    uint32_t error_count;
    
    // Calibration and filtering
    SensorCalibration_t calibration;
    MovingAverageFilter_t filter;
    
    // Timing
    uint32_t last_update_ms;
    uint32_t update_interval_ms;
} Sensor_t;

// Complete sensor array data
typedef struct {
    uint32_t left_mm;
    uint32_t front_left_mm;
    uint32_t front_right_mm;
    uint32_t right_mm;
} SensorDistances_t;

typedef struct {
    uint32_t left;
    uint32_t front_left;
    uint32_t front_right;
    uint32_t right;
} SensorRawValues_t;

// Wall detection results
typedef struct {
    bool left_wall;
    bool front_wall;
    bool right_wall;
    uint16_t left_distance_mm;
    uint16_t front_distance_mm;
    uint16_t right_distance_mm;
    float alignment_error;  // Positive = leaning right
} WallDetection_t;

// Sensor system configuration
typedef struct {
    uint32_t pwm_frequency_hz;
    uint32_t sampling_frequency_hz;
    uint16_t wall_threshold_mm;
    uint16_t alignment_threshold_mm;
    bool auto_calibration_enabled;
} SensorConfig_t;

// Function prototypes
bool Sensor_Init(SensorConfig_t *config);
bool Sensor_Calibrate(SensorID_t sensor_id, uint16_t known_distance_mm);
void Sensor_StartReading(SensorMode_t mode);
bool Sensor_IsReadingComplete(void);
void Sensor_GetDistances(SensorDistances_t *distances);
void Sensor_GetRawValues(SensorRawValues_t *raw_values);
WallDetection_t Sensor_DetectWalls(void);

// Individual sensor functions
uint16_t Sensor_GetDistance(SensorID_t sensor_id);
uint16_t Sensor_GetRawValue(SensorID_t sensor_id);
SensorStatus_t Sensor_GetStatus(SensorID_t sensor_id);
bool Sensor_IsReliable(SensorID_t sensor_id);

// Calibration functions
bool Sensor_StartCalibration(SensorID_t sensor_id);
bool Sensor_AddCalibrationPoint(SensorID_t sensor_id, uint16_t distance_mm);
bool Sensor_FinishCalibration(SensorID_t sensor_id);
void Sensor_LoadCalibration(SensorID_t sensor_id, const SensorCalibration_t *cal);
void Sensor_SaveCalibration(SensorID_t sensor_id, SensorCalibration_t *cal);

// System control
void Sensor_Update(void);  // Call from main loop
void Sensor_EmergencyStop(void);
void Sensor_Reset(void);

// Utility functions
float Sensor_GetAlignmentError(void);
bool Sensor_IsWallPresent(SensorID_t sensor_id, uint16_t threshold_mm);
uint32_t Sensor_GetUpdateRate(void);

#endif // SENSOR_H

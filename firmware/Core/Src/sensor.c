
// sensor.c - Implementation
#include "sensor.h"
#include <math.h>
#include <string.h>

// External hardware handles
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;

// System state
static Sensor_t sensors[SENSOR_COUNT];
static SensorConfig_t sensor_config;
static volatile bool reading_in_progress = false;
static uint32_t system_time_ms = 0;

// PWM control macros
#define PWM_FULL  (__HAL_TIM_GET_AUTORELOAD(&htim3))
#define PWM_SHORT (PWM_FULL/6)

// Private function prototypes
static void Sensor_InitHardware(void);
static uint16_t Sensor_ReadADC(uint8_t channel);
static void Sensor_SetEmitter(uint8_t emitter, bool state, SensorMode_t mode);
static uint16_t Sensor_ApplyCalibration(SensorID_t sensor_id, uint16_t raw_value);
static void Sensor_UpdateFilter(MovingAverageFilter_t *filter, uint16_t value);
static uint16_t Sensor_GetFilteredValue(const MovingAverageFilter_t *filter);
static void Sensor_UpdateStatus(Sensor_t *sensor);

bool Sensor_Init(SensorConfig_t *config) {
    // Copy configuration
    sensor_config = *config;
    
    // Initialize hardware
    Sensor_InitHardware();
    
    // Initialize sensor structures
    for (int i = 0; i < SENSOR_COUNT; i++) {
        Sensor_t *sensor = &sensors[i];
        memset(sensor, 0, sizeof(Sensor_t));
        
        sensor->id = (SensorID_t)i;
        sensor->adc_channel = i;  // Assuming channels 0-3
        sensor->emitter_channel = i;
        sensor->mode = SENSOR_MODE_IDLE;
        sensor->status = SENSOR_STATUS_OK;
        sensor->update_interval_ms = 1000 / config->sampling_frequency_hz;
        
        // Initialize filter
        sensor->filter.initialized = false;
        sensor->filter.index = 0;
        sensor->filter.sum = 0;
        
        // Default calibration (linear approximation)
        sensor->calibration.is_calibrated = false;
        sensor->calibration.point_count = 0;
    }
    
    return true;
}

static void Sensor_InitHardware(void) {
    // Start PWM channels
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    // Ensure all emitters are off
    for (int i = 0; i < SENSOR_COUNT; i++) {
        Sensor_SetEmitter(i, false, SENSOR_MODE_IDLE);
    }
}

static void Sensor_SetEmitter(uint8_t emitter, bool state, SensorMode_t mode) {
    uint32_t duty = 0;
    
    if (state) {
        switch (mode) {
            case SENSOR_MODE_SHORT_RANGE:
                duty = PWM_SHORT;
                break;
            case SENSOR_MODE_LONG_RANGE:
                duty = PWM_FULL;
                break;
            default:
                duty = PWM_SHORT;
                break;
        }
    }
    
    uint32_t channel = (emitter == 0) ? TIM_CHANNEL_1 :
                      (emitter == 1) ? TIM_CHANNEL_2 :
                      (emitter == 2) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
    
    __HAL_TIM_SET_COMPARE(&htim3, channel, duty);
    htim3.Instance->EGR = TIM_EGR_UG;  // Force update
}

static uint16_t Sensor_ReadADC(uint8_t channel) {
    uint32_t sum = 0;
    
    // Take multiple samples for noise reduction
    for (int i = 0; i < SENSOR_SAMPLES_PER_READING; i++) {
        HAL_ADC_Start(&hadc1);
        
        // Skip to the desired channel
        for (int ch = 0; ch <= channel; ch++) {
            HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
            if (ch == channel) {
                sum += HAL_ADC_GetValue(&hadc1);
            } else {
                HAL_ADC_GetValue(&hadc1);  // Discard
            }
        }
        
        HAL_ADC_Stop(&hadc1);
    }
    
    return (uint16_t)(sum / SENSOR_SAMPLES_PER_READING);
}

void Sensor_StartReading(SensorMode_t mode) {
    if (reading_in_progress) return;
    
    reading_in_progress = true;
    
    // Sequential reading of all sensors
    for (int sensor_id = 0; sensor_id < SENSOR_COUNT; sensor_id++) {
        Sensor_t *sensor = &sensors[sensor_id];
        
        // Turn on emitter
        Sensor_SetEmitter(sensor->emitter_channel, true, mode);
        
        // Small delay for IR LED to stabilize
        HAL_Delay(1);
        
        // Read ADC
        uint16_t raw_value = Sensor_ReadADC(sensor->adc_channel);
        
        // Turn off emitter
        Sensor_SetEmitter(sensor->emitter_channel, false, mode);
        
        // Update sensor data
        sensor->raw_adc = raw_value;
        sensor->mode = mode;
        sensor->reading_count++;
        sensor->last_update_ms = system_time_ms;
        
        // Apply filtering
        Sensor_UpdateFilter(&sensor->filter, raw_value);
        sensor->filtered_adc = Sensor_GetFilteredValue(&sensor->filter);
        
        // Convert to distance
        sensor->distance_mm = Sensor_ApplyCalibration(sensor->id, sensor->filtered_adc);
        
        // Update status
        Sensor_UpdateStatus(sensor);
    }
    
    reading_in_progress = false;
}

static void Sensor_UpdateFilter(MovingAverageFilter_t *filter, uint16_t value) {
    if (!filter->initialized) {
        // Initialize with first value
        for (int i = 0; i < SENSOR_FILTER_DEPTH; i++) {
            filter->buffer[i] = value;
        }
        filter->sum = value * SENSOR_FILTER_DEPTH;
        filter->initialized = true;
    } else {
        // Update moving average
        filter->sum -= filter->buffer[filter->index];
        filter->buffer[filter->index] = value;
        filter->sum += value;
        filter->index = (filter->index + 1) % SENSOR_FILTER_DEPTH;
    }
}

static uint16_t Sensor_GetFilteredValue(const MovingAverageFilter_t *filter) {
    if (!filter->initialized) return 0;
    return (uint16_t)(filter->sum / SENSOR_FILTER_DEPTH);
}

static uint16_t Sensor_ApplyCalibration(SensorID_t sensor_id, uint16_t raw_value) {
    Sensor_t *sensor = &sensors[sensor_id];
    
    if (!sensor->calibration.is_calibrated) {
        // Simple linear approximation if not calibrated
        // This assumes typical IR sensor behavior
        if (raw_value < 100) return SENSOR_MAX_DISTANCE_MM;
        if (raw_value > 3000) return SENSOR_MIN_DISTANCE_MM;
        
        // Linear interpolation
        return SENSOR_MAX_DISTANCE_MM - 
               ((raw_value - 100) * (SENSOR_MAX_DISTANCE_MM - SENSOR_MIN_DISTANCE_MM)) / 2900;
    }
    
    // Use polynomial calibration
    float *coeffs = sensor->calibration.polynomial_coeffs;
    float x = (float)raw_value;
    float distance = coeffs[0] + coeffs[1]*x + coeffs[2]*x*x + coeffs[3]*x*x*x;
    
    // Clamp to reasonable range
    if (distance < SENSOR_MIN_DISTANCE_MM) distance = SENSOR_MIN_DISTANCE_MM;
    if (distance > SENSOR_MAX_DISTANCE_MM) distance = SENSOR_MAX_DISTANCE_MM;
    
    return (uint16_t)distance;
}

static void Sensor_UpdateStatus(Sensor_t *sensor) {
    sensor->status = SENSOR_STATUS_OK;
    
    // Check for out of range
    if (sensor->distance_mm >= SENSOR_MAX_DISTANCE_MM || 
        sensor->distance_mm <= SENSOR_MIN_DISTANCE_MM) {
        sensor->status |= SENSOR_STATUS_OUT_OF_RANGE;
    }
    
    // Check for unreliable readings (high variance)
    // This would require tracking variance over time
    
    // Check calibration status
    if (!sensor->calibration.is_calibrated) {
        sensor->status |= SENSOR_STATUS_CALIBRATION_NEEDED;
    }
}

bool Sensor_IsReadingComplete(void) {
    return !reading_in_progress;
}

void Sensor_GetDistances(SensorDistances_t *distances) {
    distances->left_mm = sensors[SENSOR_LEFT].distance_mm;
    distances->front_left_mm = sensors[SENSOR_FRONT_LEFT].distance_mm;
    distances->front_right_mm = sensors[SENSOR_FRONT_RIGHT].distance_mm;
    distances->right_mm = sensors[SENSOR_RIGHT].distance_mm;
}

void Sensor_GetRawValues(SensorRawValues_t *raw_values) {
    raw_values->left = sensors[SENSOR_LEFT].raw_adc;
    raw_values->front_left = sensors[SENSOR_FRONT_LEFT].raw_adc;
    raw_values->front_right = sensors[SENSOR_FRONT_RIGHT].raw_adc;
    raw_values->right = sensors[SENSOR_RIGHT].raw_adc;
}

WallDetection_t Sensor_DetectWalls(void) {
    WallDetection_t result = {0};
    
    // Get current distances
    result.left_distance_mm = sensors[SENSOR_LEFT].distance_mm;
    result.front_distance_mm = (sensors[SENSOR_FRONT_LEFT].distance_mm + 
                               sensors[SENSOR_FRONT_RIGHT].distance_mm) / 2;
    result.right_distance_mm = sensors[SENSOR_RIGHT].distance_mm;
    
    // Detect walls based on threshold
    result.left_wall = result.left_distance_mm < sensor_config.wall_threshold_mm;
    result.right_wall = result.right_distance_mm < sensor_config.wall_threshold_mm;
    result.front_wall = result.front_distance_mm < sensor_config.wall_threshold_mm;
    
    // Calculate alignment error (difference between left and right front sensors)
    if (result.left_wall && result.right_wall) {
        int16_t left_front = sensors[SENSOR_FRONT_LEFT].distance_mm;
        int16_t right_front = sensors[SENSOR_FRONT_RIGHT].distance_mm;
        result.alignment_error = (float)(right_front - left_front);
    }
    
    return result;
}

uint16_t Sensor_GetDistance(SensorID_t sensor_id) {
    if (sensor_id >= SENSOR_COUNT) return 0;
    return sensors[sensor_id].distance_mm;
}

uint16_t Sensor_GetRawValue(SensorID_t sensor_id) {
    if (sensor_id >= SENSOR_COUNT) return 0;
    return sensors[sensor_id].raw_adc;
}

SensorStatus_t Sensor_GetStatus(SensorID_t sensor_id) {
    if (sensor_id >= SENSOR_COUNT) return SENSOR_STATUS_ERROR;
    return sensors[sensor_id].status;
}

bool Sensor_IsReliable(SensorID_t sensor_id) {
    if (sensor_id >= SENSOR_COUNT) return false;
    return (sensors[sensor_id].status == SENSOR_STATUS_OK);
}

bool Sensor_StartCalibration(SensorID_t sensor_id) {
    if (sensor_id >= SENSOR_COUNT) return false;
    
    Sensor_t *sensor = &sensors[sensor_id];
    sensor->calibration.point_count = 0;
    sensor->calibration.is_calibrated = false;
    sensor->mode = SENSOR_MODE_CALIBRATION;
    
    return true;
}

bool Sensor_AddCalibrationPoint(SensorID_t sensor_id, uint16_t distance_mm) {
    if (sensor_id >= SENSOR_COUNT) return false;
    
    Sensor_t *sensor = &sensors[sensor_id];
    if (sensor->calibration.point_count >= SENSOR_CALIBRATION_POINTS) return false;
    
    // Take a fresh reading
    Sensor_SetEmitter(sensor->emitter_channel, true, SENSOR_MODE_CALIBRATION);
    HAL_Delay(10);  // Longer delay for calibration
    uint16_t raw_value = Sensor_ReadADC(sensor->adc_channel);
    Sensor_SetEmitter(sensor->emitter_channel, false, SENSOR_MODE_CALIBRATION);
    
    // Store calibration point
    uint8_t idx = sensor->calibration.point_count;
    sensor->calibration.adc_values[idx] = raw_value;
    sensor->calibration.distances_mm[idx] = distance_mm;
    sensor->calibration.point_count++;
    
    return true;
}

bool Sensor_FinishCalibration(SensorID_t sensor_id) {
    if (sensor_id >= SENSOR_COUNT) return false;
    
    Sensor_t *sensor = &sensors[sensor_id];
    if (sensor->calibration.point_count < 3) return false;  // Need at least 3 points
    
    // Simple polynomial curve fitting (could be improved with least squares)
    // For now, use linear interpolation between points
    sensor->calibration.is_calibrated = true;
    sensor->mode = SENSOR_MODE_IDLE;
    
    return true;
}

void Sensor_Update(void) {
    system_time_ms++;  // Increment system time
    
    // Check if any sensors need updating based on their intervals
    for (int i = 0; i < SENSOR_COUNT; i++) {
        Sensor_t *sensor = &sensors[i];
        if ((system_time_ms - sensor->last_update_ms) >= sensor->update_interval_ms) {
            // Sensor needs update - this could trigger automatic reading
        }
    }
}

void Sensor_EmergencyStop(void) {
    // Turn off all emitters
    for (int i = 0; i < SENSOR_COUNT; i++) {
        Sensor_SetEmitter(i, false, SENSOR_MODE_IDLE);
        sensors[i].mode = SENSOR_MODE_ERROR;
    }
}

void Sensor_Reset(void) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        sensors[i].mode = SENSOR_MODE_IDLE;
        sensors[i].status = SENSOR_STATUS_OK;
        sensors[i].error_count = 0;
    }
}

float Sensor_GetAlignmentError(void) {
    WallDetection_t walls = Sensor_DetectWalls();
    return walls.alignment_error;
}

bool Sensor_IsWallPresent(SensorID_t sensor_id, uint16_t threshold_mm) {
    if (sensor_id >= SENSOR_COUNT) return false;
    return sensors[sensor_id].distance_mm < threshold_mm;
}

uint32_t Sensor_GetUpdateRate(void) {
    return sensor_config.sampling_frequency_hz;
}
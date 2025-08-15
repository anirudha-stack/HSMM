#ifndef SENSOR_CALIB_H
#define SENSOR_CALIB_H

#include <stdint.h>

/*
 * Number of IR sensors in the system.
 */
#define N_SENSORS        4    // {0 = LEFT, 1 = FRONT_LEFT, 2 = FRONT_RIGHT, 3 = RIGHT}

/*
 * Number of calibration points per sensor. Each sensor is calibrated at
 * distances: 10, 20, 30, …, 150 mm (15 points total).
 */
#define N_CAL_POINTS    15

/*
 * Size of the moving-average window (number of raw ADC samples to average).
 */
#define N_SMOOTH_SAMPLES 5

/* Sensor identifiers */
enum {
    SENSOR_LEFT        = 0,
    SENSOR_FRONT_LEFT  = 1,
    SENSOR_FRONT_RIGHT = 2,
    SENSOR_RIGHT       = 3
};

/*
 * @brief   Initialize the smoothing buffers for all sensors.
 * @details Must be called once at program startup before invoking ConvertADCtoMM().
 */
void DistanceConv_Init(void);

/*
 * @brief   Convert a raw 12-bit ADC sample into a distance in millimetres.
 * @param   sensor_id  One of {SENSOR_LEFT, SENSOR_FRONT_LEFT, SENSOR_FRONT_RIGHT, SENSOR_RIGHT}.
 * @param   raw_adc    Raw ADC reading (0..4095) from the specified sensor.
 * @return  Interpolated distance in millimetres (clamped to [10,150] mm).
 *
 * Internally, this function:
 *   1. Applies an N_SMOOTH_SAMPLES-point moving average (to reduce noise).
 *   2. Looks up into a per-sensor calibration table (adc_table[ sensor_id ][ * ])
 *      and linearly interpolates against the shared mm_table[ * ] = {10,20,…,150}.
 *
 * Note: Until you populate the adc_table[ ][ ] values in sensorCalib.c with real
 *       calibration data, this function will only clamp outputs to either 10 mm
 *       or 150 mm.
 */
uint16_t ConvertADCtoMM(uint8_t sensor_id, uint16_t raw_adc);

#endif // SENSOR_CALIB_H

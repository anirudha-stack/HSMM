#include "sensorCalib.h"
#include <stddef.h>   // for size_t

/*
 * ----------------------------------------------------------------------------
 * 1) Calibration tables for all 4 sensors.
 *
 *    Use the provided `calibRaw` data for each sensor at distances:
 *      10, 20, 30, …, 150 mm (15 points).
 *
 *    The arrays below correspond exactly to the user-supplied values:
 *      calibRaw[0] ? S1 (LEFT)
 *      calibRaw[1] ? S2 (FRONT_LEFT)
 *      calibRaw[2] ? S3 (FRONT_RIGHT)
 *      calibRaw[3] ? S4 (RIGHT)
 *
 *    Distances (mm) are shared: {10,20,30,…,150}. Replace placeholders
 *    only if you wish to recalibrate.
 * ----------------------------------------------------------------------------
 */
static const uint16_t calibRaw[N_SENSORS][N_CAL_POINTS] = {
    // S1 (LEFT)
    { 3863, 3860, 3793, 3680, 3333, 2453, 1975, 1558, 1300, 1063,  860,  720,  600,  500,  400 },
    // S2 (FRONT_LEFT)
    { 3865, 3850, 3825, 3765, 3610, 3095, 2513, 2210, 1927, 1705, 1490, 1245, 1185, 1085, 1120 },
    // S3 (FRONT_RIGHT)
    { 3866, 3850, 3832, 3795, 3730, 3530, 3025, 2595, 2258, 1982, 1762, 1523, 1435, 1312, 1230 },
    // S4 (RIGHT)
    { 3863, 3860, 3793, 3680, 3333, 2453, 1975, 1558, 1300, 1063,  860,  720,  600,  500,  400 }
};

/*
 * Shared distance table (in millimetres). The same vector of distances
 * applies to all four sensors:
 */
static const uint16_t mm_table[N_CAL_POINTS] = {
     10,  20,  30,  40,  50,
     60,  70,  80,  90, 100,
    110, 120, 130, 140, 150
};

/*
 * ----------------------------------------------------------------------------
 * 2) One moving-average buffer per sensor.
 *
 *    Each sensor_id (0..3) has a circular buffer of size N_SMOOTH_SAMPLES
 *    to implement a simple moving average over the last N_SMOOTH_SAMPLES ADC
 *    readings.
 * ----------------------------------------------------------------------------
 */
typedef struct {
    uint16_t buffer[N_SMOOTH_SAMPLES];
    uint8_t  idx;   // Next index to overwrite (0..N_SMOOTH_SAMPLES-1)
    uint32_t sum;   // Sum of all samples currently in the buffer
} SmoothBuf_t;

/* Array of smoothing buffers, one per sensor */
static SmoothBuf_t smoothBufs[N_SENSORS];

/*
 * @brief   Initialize all smoothing buffers to zero.
 * @details Call once at startup before any ConvertADCtoMM() calls.
 */
static void SmoothBufs_Init(void)
{
    for (int s = 0; s < N_SENSORS; s++) {
        for (int i = 0; i < N_SMOOTH_SAMPLES; i++) {
            smoothBufs[s].buffer[i] = 0;
        }
        smoothBufs[s].idx = 0;
        smoothBufs[s].sum = 0;
    }
}

/*
 * @brief   Feed a new raw ADC sample into sensor_id’s circular buffer.
 * @param   sensor_id  [0..N_SENSORS-1]
 * @param   newSample  Raw ADC reading (0..4095)
 * @return  The integer-averaged (smoothed) ADC over the last N_SMOOTH_SAMPLES.
 */
static uint16_t SmoothBuf_Update(uint8_t sensor_id, uint16_t newSample)
{
    SmoothBuf_t *b = &smoothBufs[sensor_id];

    // Subtract the oldest sample
    b->sum -= b->buffer[b->idx];

    // Overwrite it with the new sample
    b->buffer[b->idx] = newSample;
    b->sum += newSample;

    // Advance circular index
    b->idx = (b->idx + 1) % N_SMOOTH_SAMPLES;

    // Return integer average
    return (uint16_t)(b->sum / N_SMOOTH_SAMPLES);
}

/*
 * ----------------------------------------------------------------------------
 * 3) Generic interpolation routine: ADC ? mm
 *
 *    Given a smoothed ADC value and pointers to:
 *      - a particular sensor’s calibRaw row (sorted descending)
 *      - the shared mm_table[]    (sorted ascending)
 *
 *    both of length 'table_len', this function finds the correct interval
 *    and linearly interpolates.  If adc_val is above the highest table entry
 *    (closest distance), it clamps to mm_table[0].  If below the lowest entry
 *    (furthest distance), it clamps to mm_table[last].
 * ----------------------------------------------------------------------------
 */
static uint16_t ADC_to_mm_generic(
    uint16_t          adc_val,
    const uint16_t   *adc_table_row,
    const uint16_t   *mm_table,
    size_t            table_len
)
{
    // Clamp: if the ADC is higher than or equal to the “closest” entry ? return smallest mm
    if (adc_val >= adc_table_row[0]) {
        return mm_table[0];
    }
    // Clamp: if the ADC is lower than or equal to the “furthest” entry ? return largest mm
    if (adc_val <= adc_table_row[table_len - 1]) {
        return mm_table[table_len - 1];
    }

    // Find interval i where adc_table_row[i] = adc_val = adc_table_row[i+1]
    for (size_t i = 0; i < table_len - 1; i++) {
        uint16_t adc_hi = adc_table_row[i];
        uint16_t adc_lo = adc_table_row[i + 1];

        if (adc_val <= adc_hi && adc_val >= adc_lo) {
            uint16_t mm_hi = mm_table[i];
            uint16_t mm_lo = mm_table[i + 1];

            // Linear interpolation:
            //   fraction = (adc_val - adc_lo) / (adc_hi - adc_lo)
            //   result_mm = mm_lo + fraction * (mm_hi - mm_lo)
            uint32_t numerator   = (uint32_t)(adc_val - adc_lo) * (uint32_t)(mm_hi - mm_lo);
            uint32_t denominator = (uint32_t)(adc_hi - adc_lo);
            uint16_t result_mm   = (uint16_t)(mm_lo + (numerator / denominator));
            return result_mm;
        }
    }

    // Should never reach here if the tables are strictly sorted
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 * 4) Public API: ConvertADCtoMM
 *
 *    1) Smooth the incoming raw ADC via SmoothBuf_Update(...)
 *    2) Interpolate using the correct row of calibRaw[][] and shared mm_table[]
 * ----------------------------------------------------------------------------
 */
uint16_t ConvertADCtoMM(uint8_t sensor_id, uint16_t raw_adc)
{
    // 1) Smooth the raw ADC sample
    uint16_t smoothed_value = SmoothBuf_Update(sensor_id, raw_adc);

    // 2) Interpolate to get millimetres
    return ADC_to_mm_generic(
        smoothed_value,
        calibRaw[sensor_id],  // pointer to the correct sensor’s calibration row
        mm_table,             // shared distance array
        N_CAL_POINTS
    );
}

/*
 * ----------------------------------------------------------------------------
 * 5) Public API: DistanceConv_Init
 *
 *    Must be called once at program startup (before any ConvertADCtoMM calls).
 * ----------------------------------------------------------------------------
 */
void DistanceConv_Init(void)
{
    SmoothBufs_Init();
}

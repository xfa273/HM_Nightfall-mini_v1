/*
 * sensor_distance.h
 *
 *  Front wall sensor distance conversion (AD -> distance [mm]) using LUTs.
 *  - Individual LUTs for FL and FR
 *  - Supports non-uniform distance grid (e.g., 0..20mm at 1mm step, then coarse)
 *  - Monotonic assumption: distance increases as AD decreases
 */
#ifndef INC_SENSOR_DISTANCE_H_
#define INC_SENSOR_DISTANCE_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of LUT points per sensor
#ifndef SENSOR_DIST_LUT_MAX_POINTS
#define SENSOR_DIST_LUT_MAX_POINTS 128
#endif

// Initialize module with default coarse LUT (0..90mm, 10mm step)
// Safe to call multiple times.
void sensor_distance_init(void);

// Set/replace FL LUT
//  - mm[i]: distance in mm (monotonically increasing)
//  - ad[i]: AD counts (monotonically decreasing wrt mm)
//  - n: number of points (>=2, <= SENSOR_DIST_LUT_MAX_POINTS)
// Returns 0 on success, -1 on invalid args.
int sensor_distance_set_lut_fl(const uint16_t *mm, const uint16_t *ad, size_t n);

// Set/replace FR LUT (same contract as FL)
int sensor_distance_set_lut_fr(const uint16_t *mm, const uint16_t *ad, size_t n);

// Query current LUT sizes
size_t sensor_distance_lut_size_fl(void);
size_t sensor_distance_lut_size_fr(void);

// Convert AD -> distance [mm] using FL/FR LUTs with linear interpolation.
// If input is outside the LUT AD range, perform linear extrapolation at the nearest edge (no clipping).
// Returns distance in mm. If LUT is not initialized, default LUT is used.
float sensor_distance_from_fl(uint16_t ad_value);
float sensor_distance_from_fr(uint16_t ad_value);

#ifdef __cplusplus
}
#endif

#endif /* INC_SENSOR_DISTANCE_H_ */

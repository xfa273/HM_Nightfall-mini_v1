/*
 * sensor_distance.c
 */
#include "sensor_distance.h"
#include <string.h>

// Internal storage for LUTs (FL/FR)
static uint16_t s_mm_fl[SENSOR_DIST_LUT_MAX_POINTS];
static uint16_t s_ad_fl[SENSOR_DIST_LUT_MAX_POINTS];
static size_t   s_n_fl = 0;

static uint16_t s_mm_fr[SENSOR_DIST_LUT_MAX_POINTS];
static uint16_t s_ad_fr[SENSOR_DIST_LUT_MAX_POINTS];
static size_t   s_n_fr = 0;

// Default fine LUT: 0..20mm at 1mm steps, then 30..90mm at 10mm steps
// Values are based on provided measurements, with re-measured 4mm values applied:
//   FL@4mm=2092, FR@4mm=1957 (monotonicity preserved).
static const uint16_t s_mm_fine[] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
   10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
   20, 30, 40, 50, 60, 70, 80, 90
};
static const uint16_t s_fl_fine[] = {
    2644, 2500, 2363, 2202, 2092, 1919, 1818, 1705, 1595, 1522,
    1423, 1367, 1302, 1240, 1183, 1123, 1062, 1010,  964,  917,
     851,  558,  387,  284,  217,  168,  132,  107
};
static const uint16_t s_fr_fine[] = {
    2409, 2311, 2139, 2029, 1957, 1804, 1680, 1633, 1532, 1444,
    1337, 1308, 1226, 1163, 1126, 1057,  992,  958,  902,  845,
     794,  546,  368,  272,  203,  151,  129,  105
};
static const size_t s_n_fine = sizeof(s_mm_fine)/sizeof(s_mm_fine[0]);

static int validate_lut(const uint16_t *mm, const uint16_t *ad, size_t n)
{
    if (!mm || !ad) return -1;
    if (n < 2 || n > SENSOR_DIST_LUT_MAX_POINTS) return -1;
    // distance: strictly increasing, ad: strictly decreasing (monotonic)
    for (size_t i = 1; i < n; ++i) {
        if (!(mm[i] > mm[i-1])) return -1;
        if (!(ad[i] < ad[i-1])) return -1;
    }
    return 0;
}

void sensor_distance_init(void)
{
    // Load default fine-grained tables
    (void)sensor_distance_set_lut_fl(s_mm_fine, s_fl_fine, s_n_fine);
    (void)sensor_distance_set_lut_fr(s_mm_fine, s_fr_fine, s_n_fine);
}

int sensor_distance_set_lut_fl(const uint16_t *mm, const uint16_t *ad, size_t n)
{
    if (validate_lut(mm, ad, n) != 0) return -1;
    memcpy(s_mm_fl, mm, n * sizeof(uint16_t));
    memcpy(s_ad_fl, ad, n * sizeof(uint16_t));
    s_n_fl = n;
    return 0;
}

int sensor_distance_set_lut_fr(const uint16_t *mm, const uint16_t *ad, size_t n)
{
    if (validate_lut(mm, ad, n) != 0) return -1;
    memcpy(s_mm_fr, mm, n * sizeof(uint16_t));
    memcpy(s_ad_fr, ad, n * sizeof(uint16_t));
    s_n_fr = n;
    return 0;
}

size_t sensor_distance_lut_size_fl(void) { return s_n_fl; }
size_t sensor_distance_lut_size_fr(void) { return s_n_fr; }

// Binary search in a strictly decreasing ad[] array to find segment [i, i+1]
// such that ad[i] >= ad_in >= ad[i+1]. Assumes n>=2 and that ad[0] > ad[n-1].
// Returns index i in [0, n-2]. If outside range, returns 0 for upper extrap,
// or n-2 for lower extrap. Sets *extrap to 1 if outside, otherwise 0.
static size_t find_segment_desc(const uint16_t *ad, size_t n, uint16_t ad_in, int *extrap)
{
    if (ad_in >= ad[0]) {
        if (extrap) *extrap = 1; // upper-side extrapolation (near 0mm)
        return 0;
    }
    if (ad_in <= ad[n-1]) {
        if (extrap) *extrap = 1; // lower-side extrapolation (farther than last mm)
        return n - 2;
    }
    if (extrap) *extrap = 0;

    size_t lo = 0, hi = n - 1; // invariant: ad[lo] > ad[hi]
    while (lo + 1 < hi) {
        size_t mid = (lo + hi) / 2;
        if (ad_in > ad[mid]) {
            // Go toward larger ad (smaller mm)
            hi = mid;
        } else {
            // Go toward smaller ad (larger mm)
            lo = mid;
        }
    }
    return lo; // segment [lo, lo+1]
}

static float interpolate_mm_from_ad(const uint16_t *mm, const uint16_t *ad, size_t n, uint16_t ad_in)
{
    if (n < 2) return 0.0f;

    int extrap = 0;
    size_t i = find_segment_desc(ad, n, ad_in, &extrap);

    const float ad1 = (float)ad[i];
    const float ad2 = (float)ad[i+1];
    const float mm1 = (float)mm[i];
    const float mm2 = (float)mm[i+1];

    // Linear interpolation in (ad,mm) space. ad decreases with mm.
    const float dad = ad2 - ad1; // negative in normal case
    if (dad == 0.0f) {
        return mm1; // degenerate, return nearer endpoint
    }
    const float t = ((float)ad_in - ad1) / dad; // typically in [0,1], may be <0 or >1 when extrapolating
    return mm1 + t * (mm2 - mm1);
}

float sensor_distance_from_fl(uint16_t ad_value)
{
    if (s_n_fl < 2) {
        // Initialize defaults lazily if not done
        sensor_distance_init();
    }
    return interpolate_mm_from_ad(s_mm_fl, s_ad_fl, s_n_fl, ad_value);
}

float sensor_distance_from_fr(uint16_t ad_value)
{
    if (s_n_fr < 2) {
        sensor_distance_init();
    }
    return interpolate_mm_from_ad(s_mm_fr, s_ad_fr, s_n_fr, ad_value);
}

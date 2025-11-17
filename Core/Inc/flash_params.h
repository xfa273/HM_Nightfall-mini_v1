/*
 * flash_params.h
 *
 *  Parameters storage in Flash (separate from maze EEPROM emulation)
 */
#ifndef INC_FLASH_PARAMS_H_
#define INC_FLASH_PARAMS_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Use Sector 10 (0x080C0000) to store parameters
// Maze data uses Sector 11 (0x080E0000) via eeprom.c; keep them separated
#define FLASH_PARAMS_START_ADDRESS  (0x080C0000UL)
#define FLASH_PARAMS_SECTOR         (FLASH_SECTOR_10)

#define FLASH_PARAMS_MAGIC   (0x50415231UL)  // 'PAR1'
#define FLASH_PARAMS_VERSION (0x00010002UL)

// Flags for optional fields
#define FLASH_FLAG_FRONT_LUT_VALID   (1u << 0)
#define FLASH_FLAG_FRONT_WARP_VALID  (1u << 1)
// Individual channel warps
#define FLASH_FLAG_FRONT_WARP_FL     (1u << 2)
#define FLASH_FLAG_FRONT_WARP_FR     (1u << 3)

// Persisted parameters structure
// Note: keep fields 32-bit aligned for word programming
typedef struct {
    uint32_t magic;     // magic header
    uint32_t version;   // version code
    uint32_t length;    // total length in bytes of the stored structure
    uint32_t crc;       // simple checksum of payload (see implementation)

    // Wall sensor baseline parameters
    uint16_t base_l;
    uint16_t base_r;
    uint16_t base_f;

    uint16_t wall_offset_r;
    uint16_t wall_offset_l;
    uint16_t wall_offset_fr;
    uint16_t wall_offset_fl;

    // IMU offsets (optional)
    float imu_offset_z;

    // Optional front distance LUT (3 points for FL/FR at fixed distances)
    // Distances are stored to assert validity and allow future flexibility.
    uint32_t flags; // see FLASH_FLAG_*
    uint16_t front_lut_mm[3];  // typically {0,24,114}
    uint16_t front_lut_fl[3];  // AD at those distances
    uint16_t front_lut_fr[3];  // AD at those distances

    // Optional distance-domain warp for FRONT SUM: mm_est -> mm_true (3 anchors)
    float front_warp_x_mm_est[3];
    float front_warp_y_mm_true[3];

    // Optional distance-domain warps for FL / FR channels
    float front_warp_fl_x_mm_est[3];
    float front_warp_fl_y_mm_true[3];
    float front_warp_fr_x_mm_est[3];
    float front_warp_fr_y_mm_true[3];

    // Reserved/padding for future use
    uint32_t reserved[2];
} flash_params_t;

// Load params from flash. Returns true if valid data loaded.
bool flash_params_load(flash_params_t* out);

// Save params to flash (erase + program). Returns HAL status.
HAL_StatusTypeDef flash_params_save(const flash_params_t* in);

// Fill with reasonable defaults (zeros)
void flash_params_defaults(flash_params_t* out);

#ifdef __cplusplus
}
#endif

#endif /* INC_FLASH_PARAMS_H_ */

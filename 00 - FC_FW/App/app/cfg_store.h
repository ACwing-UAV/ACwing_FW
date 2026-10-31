/**
 * @file  cfg_store.h
 * @brief Persistent configuration: parameters, drone name, calibrations and
 *        one backup per calibration type (HANDOVER_FW.md §8, §10).
 *
 * Two flash pages used alternately (A/B). Each save writes the whole image,
 * with a sequence number and a CRC, to the page NOT holding the latest valid
 * image: a reset or power loss during a save leaves the previous image valid.
 * On a bad CRC everywhere the defaults are used.
 *
 * The flash access is a small backend (cfg_flash_*), implemented for the
 * STM32WB55 in bsp/cfg_flash_wb.c (CPU2-safe) and in RAM by the host test.
 */
#ifndef CFG_STORE_H
#define CFG_STORE_H

#include <stdint.h>
#include "params.h"
#include "calib_mgr.h"

#define CFG_MAGIC        0x43574341u    /**< "ACWC" */
#define CFG_VERSION      2u
#define CFG_PARAM_SLOTS  48u            /**< room for parameters added later      */

/** one parameter: stored by id, so the table can grow without a migration */
typedef struct {
    uint16_t id;                     /**< 0 = empty slot                           */
    uint16_t rsv;
    float    value;
} cfg_param_t;

/** Version 2. Version 1 (positional parameters, no battery) is migrated by
 *  cfg_load(); the next save writes version 2. */
typedef struct {
    uint32_t    magic;
    uint16_t    version;
    uint16_t    size;
    uint32_t    seq;
    cfg_param_t params[CFG_PARAM_SLOTS];
    char        name[16];
    cal_set_t   cal;                 /**< saved calibrations                       */
    cal_set_t   bak;                 /**< previous set, per type (CAL_RESTORE)     */
    uint32_t    bak_mask;            /**< bit (type-1): bak holds a set for it      */
    float       batt_used_mah;       /**< charge used in the last flight            */
    float       batt_used_scale_i;   /**< current scale it was counted with         */
    uint8_t     reserved[56];        /**< zero; for later fields                    */
    uint32_t    crc;                 /**< CRC-32 of everything above               */
} cfg_image_t;

/** parameters <-> slots (unknown ids ignored, missing ids -> default) */
void cfg_params_put(cfg_image_t *img, const float values[PARAM_COUNT]);
void cfg_params_get(const cfg_image_t *img, float values[PARAM_COUNT]);

/* ---- backend ---- */
#define CFG_PAGES 2
const void *cfg_flash_page(int page);                         /**< memory-mapped page */
int  cfg_flash_write(int page, const void *data, uint32_t len); /**< erase + program; 0 = ok */

/* ---- store ---- */
uint32_t cfg_crc32(const void *data, uint32_t len);
/** latest valid image (version 1 converted); returns 0 if found, -1 if none
 *  (out left untouched) */
int  cfg_load(cfg_image_t *out);
/** writes img (seq and crc filled here) to the other page; 0 = ok */
int  cfg_save(cfg_image_t *img);

#endif /* CFG_STORE_H */

/*
 * cfg.h - settings kept in non-volatile memory (calibration and CAN node).
 */
#pragma once
#include <stdint.h>

typedef struct {
    uint32_t magic;
    float enc_offset;       /* electrical angle offset, rad */
    int32_t enc_dir;        /* +1 or -1 */
    int32_t cur_sign;       /* current sense polarity, +1 or -1 */
    uint32_t node_id;       /* CAN node */
    uint32_t cal_valid;     /* 1 once calibration has succeeded */
    uint32_t pad;           /* keeps the size a multiple of 8 for flash */
    uint32_t check;
} cfg_t;

extern cfg_t cfg;

void cfg_defaults(void);
void cfg_load(void);
int cfg_save(void);

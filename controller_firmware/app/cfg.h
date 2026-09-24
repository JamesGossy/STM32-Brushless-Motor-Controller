#pragma once
#include <stdint.h>

typedef struct {
    uint32_t magic;
    float enc_offset;
    int32_t enc_dir;
    int32_t cur_sign;
    uint32_t node_id;
    uint32_t cal_valid;
    uint32_t pad;
    uint32_t check;
} cfg_t;

extern cfg_t cfg;

void cfg_defaults(void);
void cfg_load(void);
int cfg_save(void);

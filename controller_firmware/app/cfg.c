/*
 * cfg.c - load and save the settings block through the HAL's NV storage.
 */
#include "cfg.h"
#include "hal.h"
#include "config.h"

#define CFG_MAGIC 0xF0C0CAFEu

cfg_t cfg;

/* FNV-style hash over every field except the check word itself. */
static uint32_t checksum(const cfg_t *c)
{
    const uint32_t *w = (const uint32_t *)c;
    uint32_t h = 0x12345678u;
    for (unsigned i = 0; i < sizeof *c / 4 - 1; i++)
        h = (h ^ w[i]) * 16777619u;
    return h;
}

/* Uncalibrated defaults. */
void cfg_defaults(void)
{
    cfg.magic = 0;
    cfg.enc_offset = 0.0f;
    cfg.enc_dir = 1;
    cfg.cur_sign = 1;
    cfg.node_id = CAN_NODE_ID;
    cfg.cal_valid = 0;
    cfg.pad = 0;
}

/* Load settings, falling back to defaults if missing or corrupt. */
void cfg_load(void)
{
    if (!hal_nv_read(&cfg, sizeof cfg) || cfg.magic != CFG_MAGIC || cfg.check != checksum(&cfg))
        cfg_defaults();
}

/* Save settings. Returns 1 on success. */
int cfg_save(void)
{
    cfg.magic = CFG_MAGIC;
    cfg.check = checksum(&cfg);
    return hal_nv_write(&cfg, sizeof cfg);
}

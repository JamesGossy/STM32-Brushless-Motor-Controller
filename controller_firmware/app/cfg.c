#include "cfg.h"
#include "config.h"
#include "hal.h"

#define CFG_MAGIC 0xF0C0CAFEu

cfg_t cfg;

static uint32_t cfg_check(const cfg_t *c)
{
    const uint32_t *w = (const uint32_t *)c;
    uint32_t s = 0x12345678u;
    for (int i = 0; i < 7; i++) s = (s ^ w[i]) * 16777619u;
    return s;
}

void cfg_defaults(void)
{
    cfg.magic = 0;
    cfg.cal_valid = 0;
    cfg.enc_offset = 0.0f;
    cfg.enc_dir = 1;
    cfg.cur_sign = 1;
    cfg.node_id = CAN_NODE_ID;
    cfg.pad = 0;
}

void cfg_load(void)
{
    if (!hal_nv_read(&cfg, sizeof cfg) || cfg.magic != CFG_MAGIC || cfg.check != cfg_check(&cfg))
        cfg_defaults();
}

int cfg_save(void)
{
    cfg.magic = CFG_MAGIC;
    cfg.check = cfg_check(&cfg);
    return hal_nv_write(&cfg, sizeof cfg);
}

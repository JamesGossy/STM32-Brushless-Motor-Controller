/*
 * test_cfg.c - unit tests for settings storage.
 */
#include "test.h"
#include "cfg.h"
#include "hal.h"
#include "config.h"

/* Empty storage loads uncalibrated defaults. */
static void defaults_without_nv(void)
{
    cfg.cal_valid = 1;
    cfg_load();                         /* sim NV starts empty */
    CHECK(cfg.cal_valid == 0 && cfg.enc_dir == 1 && cfg.cur_sign == 1 && cfg.node_id == CAN_NODE_ID);
}

/* Saved settings load back unchanged. */
static void save_load_roundtrip(void)
{
    cfg_defaults();
    cfg.enc_offset = 1.25f;
    cfg.enc_dir = -1;
    cfg.cur_sign = -1;
    cfg.cal_valid = 1;
    cfg.node_id = 9;
    CHECK(cfg_save());
    cfg_defaults();
    cfg_load();
    CHECK(cfg.cal_valid == 1 && cfg.enc_dir == -1 && cfg.cur_sign == -1 && cfg.node_id == 9);
    CHECK_NEAR(cfg.enc_offset, 1.25, 1e-7);
}

/* A corrupted block is rejected by the checksum. */
static void corruption_detected(void)
{
    cfg_defaults();
    cfg.cal_valid = 1;
    cfg_save();
    cfg_t raw;
    hal_nv_read(&raw, sizeof raw);
    raw.enc_offset += 0.1f;             /* flip bits without fixing the checksum */
    hal_nv_write(&raw, sizeof raw);
    cfg_load();
    CHECK(cfg.cal_valid == 0);
}

/* The block size suits 64-bit flash programming. */
static void struct_is_flash_friendly(void)
{
    CHECK(sizeof(cfg_t) % 8 == 0);      /* flash programs double words */
}

int main(void)
{
    test_case t[] = {T(defaults_without_nv), T(save_load_roundtrip), T(corruption_detected), T(struct_is_flash_friendly)};
    return run_tests(t, sizeof t / sizeof t[0]);
}

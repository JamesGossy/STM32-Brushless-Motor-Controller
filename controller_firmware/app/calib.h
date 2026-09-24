/*
 * calib.h - automatic encoder and current sense calibration.
 */
#pragma once

void cal_start(void);
void cal_run(float ia_raw, float raw_angle);

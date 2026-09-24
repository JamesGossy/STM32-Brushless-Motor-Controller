#pragma once
#include <stdint.h>

enum { ST_BOOT, ST_IDLE, ST_CAL, ST_RUN, ST_FAULT };
enum { MODE_TORQUE = 1, MODE_SPEED = 2 };
enum { REQ_NONE, REQ_IDLE, REQ_TORQUE, REQ_SPEED, REQ_CAL };

#define F_OVERCURRENT  (1u << 0)
#define F_OVERVOLT     (1u << 1)
#define F_UNDERVOLT    (1u << 2)
#define F_OVERTEMP     (1u << 3)
#define F_DRV          (1u << 4)
#define F_CURRENT_SUM  (1u << 5)
#define F_CAL          (1u << 6)
#define F_DRV_INIT     (1u << 7)
#define F_OVERSPEED    (1u << 8)
#define F_NOT_CAL      (1u << 9)
#define F_OFFSET       (1u << 10)
#define F_ENCODER      (1u << 11)
#define F_NAN          (1u << 12)
#define F_STALL        (1u << 13)

typedef struct {
    volatile uint8_t state, mode, req, cal_done;
    volatile uint16_t faults;
    volatile float cmd_iq, cmd_speed, i_limit, speed_limit;
    volatile float ia, ib, ic, id, iq, vd, vq, id_ref, iq_ref, speed_ref;
    volatile float vbus, theta_e, theta_m, omega_m;
    volatile float t_fet, t_amb;
} foc_t;

extern foc_t foc;

void foc_init(void);
void foc_isr(void);
void foc_fault(uint16_t f);
void foc_clear_faults(void);
uint32_t foc_isr_count(void);
void foc_pll_reset(float th);

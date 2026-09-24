/*
 * fdcan.h - FDCAN2 as classic CAN at 1 Mbit/s.
 */
#pragma once
#include <stdint.h>

void fdcan_init(void);
int fdcan_send(uint32_t id, const uint8_t *data, uint8_t len);
int fdcan_recv(uint32_t *id, uint8_t *data, uint8_t *len);

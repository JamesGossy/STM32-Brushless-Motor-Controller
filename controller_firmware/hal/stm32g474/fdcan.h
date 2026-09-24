#pragma once
#include <stdint.h>

void fdcan_init(void);
int fdcan_send(uint32_t id, const uint8_t *d, uint8_t len);
int fdcan_recv(uint32_t *id, uint8_t *d, uint8_t *len);

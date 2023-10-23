#ifndef CAN_RTX_H
#define CAN_RTX_H

#include "stdint.h"
#include "chassis.h"
#include "main.h"

void set_motor_current_can2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);

#endif
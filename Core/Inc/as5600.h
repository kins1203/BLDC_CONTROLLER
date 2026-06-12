#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C address (7-bit = 0x36) */
#define AS5600_I2C_ADDR       (0x36 << 1)

/* Register map */
#define AS5600_RAW_ANGLE_MSB  0x0E
#define AS5600_RAW_ANGLE_LSB  0x0F

typedef struct
{
    I2C_HandleTypeDef *hi2c;

    uint16_t raw_angle;   // 0..4095
    float angle_deg;      // 0..360
    float angle_rad;      // 0..2PI

} AS5600_Handle_t;

/* API */
HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef AS5600_ReadRaw(AS5600_Handle_t *h);

float AS5600_GetAngleDeg(AS5600_Handle_t *h);
float AS5600_GetAngleRad(AS5600_Handle_t *h);

#endif /* __AS5600_H */

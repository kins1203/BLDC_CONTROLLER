#include "as5600.h"
#include <math.h>

HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *h, I2C_HandleTypeDef *hi2c)
{
    if (h == NULL || hi2c == NULL)
        return HAL_ERROR;

    h->hi2c = hi2c;
    h->raw_angle = 0;
    h->angle_deg = 0.0f;
    h->angle_rad = 0.0f;

    return HAL_OK;
}

/* đọc raw angle 12-bit */
HAL_StatusTypeDef AS5600_ReadRaw(AS5600_Handle_t *h)
{
    uint8_t buf[2];

    if (HAL_I2C_Mem_Read(h->hi2c,
                          AS5600_I2C_ADDR,
                          AS5600_RAW_ANGLE_MSB,
                          I2C_MEMADD_SIZE_8BIT,
                          buf,
                          2,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    h->raw_angle = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;

    return HAL_OK;
}

/* góc độ */
float AS5600_GetAngleDeg(AS5600_Handle_t *h)
{
    if (AS5600_ReadRaw(h) != HAL_OK)
        return h->angle_deg;

    h->angle_deg = (h->raw_angle * 360.0f) / 4096.0f;
    return h->angle_deg;
}

/* góc radian */
float AS5600_GetAngleRad(AS5600_Handle_t *h)
{
    if (AS5600_ReadRaw(h) != HAL_OK)
        return h->angle_rad;

    h->angle_rad = (h->raw_angle * 2.0f * M_PI) / 4096.0f;
    return h->angle_rad;
}

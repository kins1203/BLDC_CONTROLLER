#ifndef __DRV8301_H
#define __DRV8301_H

#include "stm32f4xx_hal.h"
#include <stdint.h>


/* ================== CONFIG ====================== */

#define DRV8301_SPI_TIMEOUT   10   // ms

/* ================== REGISTER MAP ================ */

#define DRV8301_REG_STATUS_1      0x00
#define DRV8301_REG_STATUS_2      0x01
#define DRV8301_REG_CONTROL_1     0x02
#define DRV8301_REG_CONTROL_2     0x03

/* ================== HANDLE ====================== */

typedef struct
{
    SPI_HandleTypeDef *hspi;

    /* CS pin */
    GPIO_TypeDef *CS_Port;
    uint16_t      CS_Pin;

    /* EN_GATE pin */
    GPIO_TypeDef *EN_Port;
    uint16_t      EN_Pin;

    uint16_t tx_buf;
    uint16_t rx_buf;

    volatile uint8_t dma_done;
} DRV8301_HandleTypeDef;

/* ================== API ========================= */

void DRV8301_Init(DRV8301_HandleTypeDef *drv);

/* EN_GATE control */
void DRV8301_EnableGate(DRV8301_HandleTypeDef *drv);
void DRV8301_DisableGate(DRV8301_HandleTypeDef *drv);

/* Register access */
HAL_StatusTypeDef DRV8301_WriteReg(
    DRV8301_HandleTypeDef *drv,
    uint8_t reg,
    uint16_t data
);

HAL_StatusTypeDef DRV8301_ReadReg(
    DRV8301_HandleTypeDef *drv,
    uint8_t reg,
    uint16_t *data
);
/* ========== PWM MODE ========== */
typedef enum
{
    DRV8301_PWM_6X = 0,
    DRV8301_PWM_3X = 1
} DRV8301_PwmMode_t;

/* ========== OC MODE =========== */
typedef enum
{
    DRV8301_OC_LATCH_SHUTDOWN = 0,
    DRV8301_OC_REPORT_ONLY   = 1,
    DRV8301_OC_DISABLE       = 2,
    DRV8301_OC_CYCLE_BY_CYCLE= 3
} DRV8301_OcMode_t;

/* ========== CSA GAIN ========== */
typedef enum
{
    DRV8301_GAIN_10  = 0,
    DRV8301_GAIN_20  = 1,
    DRV8301_GAIN_40  = 2,
    DRV8301_GAIN_80  = 3
} DRV8301_CsaGain_t;

/* ========== CONFIG STRUCT ===== */

typedef struct
{
    DRV8301_PwmMode_t  pwm_mode;
    DRV8301_OcMode_t   oc_mode;
    DRV8301_CsaGain_t  csa_gain;
} DRV8301_Config_t;

/* API */
HAL_StatusTypeDef DRV8301_Configure(
    DRV8301_HandleTypeDef *drv,
    DRV8301_Config_t *cfg
);
/* DMA callback hook */
void DRV8301_TxRxCpltCallback(DRV8301_HandleTypeDef *drv);

#endif /* __DRV8301_H */

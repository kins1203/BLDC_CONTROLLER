#include "drv8301.h"

/* ================== GPIO ======================== */

static inline void DRV8301_CS_LOW(DRV8301_HandleTypeDef *drv)
{
    HAL_GPIO_WritePin(drv->CS_Port, drv->CS_Pin, GPIO_PIN_RESET);
}

static inline void DRV8301_CS_HIGH(DRV8301_HandleTypeDef *drv)
{
    HAL_GPIO_WritePin(drv->CS_Port, drv->CS_Pin, GPIO_PIN_SET);
}

/* ================== EN_GATE ===================== */

void DRV8301_EnableGate(DRV8301_HandleTypeDef *drv)
{
    HAL_GPIO_WritePin(drv->EN_Port, drv->EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);   // cho charge pump ổn định
}

void DRV8301_DisableGate(DRV8301_HandleTypeDef *drv)
{
    HAL_GPIO_WritePin(drv->EN_Port, drv->EN_Pin, GPIO_PIN_RESET);
}

/* ================== INIT ======================== */

void DRV8301_Init(DRV8301_HandleTypeDef *drv)
{
    DRV8301_CS_HIGH(drv);
    DRV8301_DisableGate(drv);   // an toàn khi init
    drv->dma_done = 0;
}

/* ================== TRANSFER ==================== */

static HAL_StatusTypeDef DRV8301_TransferDMA(
    DRV8301_HandleTypeDef *drv,
    uint16_t tx,
    uint16_t *rx
)
{
    drv->dma_done = 0;
    drv->tx_buf = tx;

    DRV8301_CS_LOW(drv);

    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_DMA(
        drv->hspi,
        (uint8_t *)&drv->tx_buf,
        (uint8_t *)&drv->rx_buf,
        1
    );

    if (ret != HAL_OK)
    {
        DRV8301_CS_HIGH(drv);
        return ret;
    }

    uint32_t tick = HAL_GetTick();
    while (!drv->dma_done)
    {
        if ((HAL_GetTick() - tick) > DRV8301_SPI_TIMEOUT)
        {
            DRV8301_CS_HIGH(drv);
            return HAL_TIMEOUT;
        }
    }

    DRV8301_CS_HIGH(drv);
    *rx = drv->rx_buf;
    return HAL_OK;
}
HAL_StatusTypeDef DRV8301_Configure(
    DRV8301_HandleTypeDef *drv,
    DRV8301_Config_t *cfg
)
{
    uint16_t ctrl1 = 0;
    uint16_t ctrl2 = 0;

    /* CONTROL1 */
    ctrl1 |= ((cfg->pwm_mode & 0x03) << 7);
    ctrl1 |= ((cfg->oc_mode  & 0x03) << 5);
    ctrl1 |= ((cfg->csa_gain & 0x07) << 2);

    /* CONTROL2: để mặc định an toàn */
    ctrl2 = 0x000;

    HAL_StatusTypeDef ret;
    ret = DRV8301_WriteReg(drv, DRV8301_REG_CONTROL_1, ctrl1);
    if (ret != HAL_OK) return ret;

    ret = DRV8301_WriteReg(drv, DRV8301_REG_CONTROL_2, ctrl2);
    return ret;
}

/* ================== WRITE ======================= */

HAL_StatusTypeDef DRV8301_WriteReg(
    DRV8301_HandleTypeDef *drv,
    uint8_t reg,
    uint16_t data
)
{
    uint16_t frame = (0 << 15) |
                     ((reg & 0x07) << 11) |
                     (data & 0x07FF);

    uint16_t dummy;
    return DRV8301_TransferDMA(drv, frame, &dummy);
}

/* ================== READ ======================== */

HAL_StatusTypeDef DRV8301_ReadReg(
    DRV8301_HandleTypeDef *drv,
    uint8_t reg,
    uint16_t *data
)
{
    uint16_t frame = (1 << 15) | ((reg & 0x07) << 11);
    uint16_t rx;

    HAL_StatusTypeDef ret = DRV8301_TransferDMA(drv, frame, &rx);
    if (ret == HAL_OK)
    {
        *data = rx & 0x07FF;
    }
    return ret;
}

/* ================== DMA CALLBACK ================ */

void DRV8301_TxRxCpltCallback(DRV8301_HandleTypeDef *drv)
{
    drv->dma_done = 1;
}

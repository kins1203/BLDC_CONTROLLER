/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdarg.h>      // <-- BẮT BUỘC cho va_start / va_end
#include "usbd_cdc_if.h" // <-- BẮT BUỘC cho CDC_Transmit_FS
#include "command.h"
#define CDC_LOG_BUF_SIZE 256
#include "math.h"
#include "drv8301.h"
#include <stdio.h>
#include <math.h>
#include "svpwm.h"
#include "as5600.h"
#include <ctype.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DC_BUS_VOLTAGE 12.0f
#define PWM_ARR 100
#define OPENLOOP_VOLTAGE 4.11f // V (bắt đầu nhỏ, 2–4V)
#define OPENLOOP_SPEED 10.0f   // rad/s điện
#define TWO_PI 6.283185307f
#define M_PI TWO_PI / 2.0f
/* motor params */
#define POLE_PAIRS 7         // ví dụ, bạn sửa theo motor
#define CTRL_FREQ_HZ 1000.0f // loop tốc độ 1 kHz
#define DT_CTRL (1.0f / CTRL_FREQ_HZ)

/* encoder */
float mech_angle = 0.0f; // rad
float mech_angle_prev = 0.0f;
float mech_speed = 0.0f; // rad/s

/* electrical */
float elec_offset = 0.0f; // rad
float elec_angle = 0.0f;

/* speed control */
float speed_ref = 50.0f; // rad/s
float Uq = 0.0f;

/* limit */
#define UQ_MAX 6.0f // V, tùy bus

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
#define LED_PIN GPIO_PIN_2
#define LED_PORT GPIOB
#define EN_PIN GPIO_PIN_12
#define EN_PORT GPIOB

#define FAULT_PIN GPIO_PIN_12
#define FAULT_PORT GPIOD

/* ===== HALL GPIO ===== */
#define HALL_A_PORT GPIOB
#define HALL_A_PIN GPIO_PIN_4

#define HALL_B_PORT GPIOB
#define HALL_B_PIN GPIO_PIN_5

#define HALL_C_PORT GPIOA
#define HALL_C_PIN GPIO_PIN_15
#define DEG2RAD(x) ((x) * M_PI / 180.0f)
/* điện áp calib – vừa đủ kéo rotor */
#define CALIB_VOLTAGE 1.0f  // chỉnh theo motor
#define CALIB_DELAY_MS 2000 // thời gian chờ rotor ổn định
/* thử các giá trị: 30, 60, 90, 120, 150 */
static float hall_angle_offset = DEG2RAD(90.0f);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//================= Khai báo các đối tượng ======================
DRV8301_HandleTypeDef hdrv8301;
DRV8301_HandleTypeDef hdrv8301 =
    {
        .hspi = &hspi3,

        .CS_Port = GPIOC,
        .CS_Pin = GPIO_PIN_13,

        .EN_Port = GPIOB,
        .EN_Pin = GPIO_PIN_12 // ví dụ
};
DRV8301_Config_t drv_cfg =
    {
        .pwm_mode = DRV8301_PWM_6X,
        .oc_mode = DRV8301_OC_REPORT_ONLY,
        .csa_gain = DRV8301_GAIN_10};
SVPWM_Handle_t hsvpwm =
    {
        .pwm_period = PWM_ARR,
        .Udc = 12.0f};
AS5600_Handle_t as5600;
CmdParsed_t cmd;
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
  if (hspi == &hspi3)
  {
    DRV8301_TxRxCpltCallback(&hdrv8301);
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//===================== Khai báo các biến ===================
extern uint8_t cdc_rx_buf[64];
extern volatile uint8_t cdc_rx_len;
extern volatile uint8_t cdc_rx_flag;
long cnt = 0;
static uint32_t last_tick = 0;
uint32_t last_time = 0;
/* Hall ABC -> sector (0..5) */
static const int8_t hall_to_sector[8] =
    {
        -1, // 000 invalid
        0,  // 001
        4,  // 010
        5,  // 011
        2,  // 100
        1,  // 101
        3,  // 110
        -1  // 111 invalid
};
/* Góc giữa mỗi sector 60° */
static const float sector_angle[12] =
    {
        0.0f,                // 0
        M_PI / 6.0f,         // 30
        M_PI / 3.0f,         // 60
        M_PI / 2.0f,         // 90
        2.0f * M_PI / 3.0f,  // 120
        5.0f * M_PI / 6.0f,  // 150
        M_PI,                // 180
        7.0f * M_PI / 6.0f,  // 210
        4.0f * M_PI / 3.0f,  // 240
        3.0f * M_PI / 2.0f,  // 270
        5.0f * M_PI / 3.0f,  // 300
        11.0f * M_PI / 6.0f, // 330
};
float hall_voltage = 7.8f; // điện áp điều khiển
static inline uint8_t Hall_ReadABC(void)
{
  uint8_t a = HAL_GPIO_ReadPin(HALL_A_PORT, HALL_A_PIN) ? 1 : 0;
  uint8_t b = HAL_GPIO_ReadPin(HALL_B_PORT, HALL_B_PIN) ? 1 : 0;
  uint8_t c = HAL_GPIO_ReadPin(HALL_C_PORT, HALL_C_PIN) ? 1 : 0;

  return (a << 2) | (b << 1) | c;
}
uint8_t hall = 0;
float angle = 0.0;
uint8_t buffer[64];
void CDC_Log(const char *fmt, ...)
{
  static char buf[CDC_LOG_BUF_SIZE];
  int len = 0;

  /* Prefix */
  len += snprintf(buf + len, CDC_LOG_BUF_SIZE - len, "LOG, ");

  /* Format nội dung */
  va_list args;
  va_start(args, fmt);
  len += vsnprintf(buf + len, CDC_LOG_BUF_SIZE - len, fmt, args);
  va_end(args);

  /* Kết thúc dòng */
  len += snprintf(buf + len, CDC_LOG_BUF_SIZE - len, "\r\n");

  /* Gửi qua USB CDC */
  if (len > 0)
  {
    CDC_Transmit_FS((uint8_t *)buf, len);
  }
}
void Motor_Hall_SVPWM_Run(void)
{
  hall = Hall_ReadABC();
  int8_t sector = hall_to_sector[hall];

  if (sector < 0)
  {
    /* Hall lỗi (000 hoặc 111) → tắt PWM cho an toàn */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    return;
  }

  /* Góc điện trung tâm sector */
  angle = sector_angle[sector];
  /* wrap về 0..2π */
  if (angle >= 2.0f * M_PI)
    angle -= 2.0f * M_PI;
  /* Vector điện áp */
  float Ualpha = hall_voltage * cosf(angle);
  float Ubeta = hall_voltage * sinf(angle);

  /* SVPWM */
  float dutyA, dutyB, dutyC;
  SVPWM_Compute(&hsvpwm, Ualpha, Ubeta, &dutyA, &dutyB, &dutyC);

  /* Xuất PWM */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyA * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyB * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyC * hsvpwm.pwm_period);
  CDC_Log("a=%.2f, b=%.2f,c=%.2f", dutyA / 3.0f, dutyB / 3.0f, dutyC / 3.0f);
}

void Motor_OpenLoop_SVPWM_Run(void)
{
  uint32_t now = HAL_GetTick();
  float dt = (now - last_tick) * 0.001f; // ms → s
  last_tick = now;

  /* 1. cập nhật góc điện */
  elec_angle += OPENLOOP_SPEED * dt;
  if (elec_angle > TWO_PI)
    elec_angle -= TWO_PI;

  /* 2. tạo vector điện áp */
  float Ualpha = OPENLOOP_VOLTAGE * cosf(elec_angle);
  float Ubeta = OPENLOOP_VOLTAGE * sinf(elec_angle);

  /* 3. SVPWM */
  float dutyA, dutyB, dutyC;
  SVPWM_Compute(&hsvpwm, Ualpha, Ubeta, &dutyA, &dutyB, &dutyC);

  /* 4. xuất PWM */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyA * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyB * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyC * hsvpwm.pwm_period);
  CDC_Log("a=%.2f, b=%.2f,c=%.2f", dutyA / 3.0f, dutyB / 3.0f, dutyC / 3.0f);
}
static void Motor_Lock_ElectricalAngle(float angle)
{
  float Ualpha = CALIB_VOLTAGE * cosf(angle);
  float Ubeta = CALIB_VOLTAGE * sinf(angle);

  float dutyA, dutyB, dutyC;
  SVPWM_Compute(&hsvpwm, Ualpha, Ubeta, &dutyA, &dutyB, &dutyC);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyA * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyB * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyC * hsvpwm.pwm_period);
}
float Motor_Calib_HallOffset(void)
{
  uint8_t hall;
  int8_t sector;
  float offset;

  CDC_Log("HALL CALIB START");

  /* 1. Khóa rotor tại góc điện 0° */
  Motor_Lock_ElectricalAngle(0.0f);
  HAL_Delay(CALIB_DELAY_MS);

  /* 2. Đọc Hall */
  hall = Hall_ReadABC();
  sector = hall_to_sector[hall];

  if (sector < 0)
  {
    CDC_Log("HALL CALIB FAIL, hall=%d", hall);
    return 0.0f;
  }

  /* 3. Tính offset:
   * Ta muốn angle + offset = center của sector
   * center = sector * 60° + 30°
   */
  offset = (sector * (M_PI / 3.0f)) + (M_PI / 6.0f);

  /* 4. Wrap về 0..2π */
  if (offset >= TWO_PI)
    offset -= TWO_PI;

  CDC_Log("HALL CALIB OK, hall=%d sector=%d offset=%.1f deg",
          hall,
          sector,
          offset * 180.0f / M_PI);

  return offset;
}
float Encoder_GetAngleRad(void)
{
  return AS5600_GetAngleRad(&as5600); // 0..2PI
}

float Encoder_GetSpeedRad(void)
{
  float dtheta;

  mech_angle = Encoder_GetAngleRad();
  dtheta = mech_angle - mech_angle_prev;

  /* unwrap */
  if (dtheta > M_PI)
    dtheta -= 2.0f * M_PI;
  if (dtheta < -M_PI)
    dtheta += 2.0f * M_PI;

  mech_speed = dtheta / DT_CTRL;
  mech_angle_prev = mech_angle;

  return mech_speed;
}
#define CALIB_VOLTAGE 5.0f
#define CALIB_DELAY_MS 500
typedef struct
{
  float kp, ki;
  float integral;
} PID_t;

PID_t pid_speed = {
    .kp = 0.02f,
    .ki = 0.5f,
    .integral = 0.0f};

float PID_Speed_Run(PID_t *pid, float ref, float fb)
{
  float err = ref - fb;

  pid->integral += err * DT_CTRL;

  float out = pid->kp * err + pid->ki * pid->integral;

  /* clamp */
  if (out > UQ_MAX)
    out = UQ_MAX;
  if (out < -UQ_MAX)
    out = -UQ_MAX;

  return out;
}
void Motor_SpeedControl_Loop(void)
{
  /* 1. speed feedback */
  float speed_fb = Encoder_GetSpeedRad();

  /* 2. PID */
  Uq = PID_Speed_Run(&pid_speed, speed_ref, speed_fb);

  /* 3. electrical angle */
  elec_angle = POLE_PAIRS * mech_angle + elec_offset;

  if (elec_angle > 2.0f * M_PI)
    elec_angle -= 2.0f * M_PI;

  /* 4. SVPWM (Id = 0) */
  float Ualpha = -Uq * sinf(elec_angle);
  float Ubeta = Uq * cosf(elec_angle);

  float dA, dB, dC;
  SVPWM_Compute(&hsvpwm, Ualpha, Ubeta, &dA, &dB, &dC);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dA * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dB * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dC * hsvpwm.pwm_period);
}

void Motor_Calib_EncoderOffset(void)
{
  float mech;

  CDC_Log("ENCODER CALIB START");

  /* 1. khóa rotor tại góc điện 0 */
  float Ualpha = CALIB_VOLTAGE;
  float Ubeta = 0.0f;

  float dA, dB, dC;
  SVPWM_Compute(&hsvpwm, Ualpha, Ubeta, &dA, &dB, &dC);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dA * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dB * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dC * hsvpwm.pwm_period);

  HAL_Delay(CALIB_DELAY_MS);

  /* 2. đọc góc cơ */
  mech = Encoder_GetAngleRad();

  /* 3. tính offset */
  elec_offset = -POLE_PAIRS * mech;

  /* wrap */
  while (elec_offset < 0)
    elec_offset += 2.0f * M_PI;
  while (elec_offset > 2.0f * M_PI)
    elec_offset -= 2.0f * M_PI;

  CDC_Log("ENC CALIB DONE, offset = %.2f deg",
          elec_offset * 180.0f / M_PI);
}
#define CALIB_VOLTAGE 1.8f                 // nhỏ, không giật mạnh
#define CALIB_ANGLE (2.0f * M_PI / 180.0f) // ±20° điện
#define CALIB_DELAY_MS 5

void Motor_Calib_EncoderOffset_Bidir(void)
{
  float mech_fwd, mech_rev;
  float angle = 0;

  CDC_Log("ENC CALIB BIDIR START");
  /* --- BƯỚC 1: QUAY TIẾN --- */
  while (angle <= M_PI * 8)
  {
    angle += CALIB_ANGLE;
    float Ualpha = CALIB_VOLTAGE * cosf(angle);
    float Ubeta = CALIB_VOLTAGE * sinf(angle);

    float dA, dB, dC;
    SVPWM_Compute(&hsvpwm, Ualpha, Ubeta, &dA, &dB, &dC);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dA * hsvpwm.pwm_period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dB * hsvpwm.pwm_period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dC * hsvpwm.pwm_period);

    HAL_Delay(CALIB_DELAY_MS);
    mech_fwd = Encoder_GetAngleRad();
  }
  HAL_Delay(500);
  while (angle > 0.0f)
  {
    angle -= CALIB_ANGLE;
    float Ualpha = CALIB_VOLTAGE * cosf(angle);
    float Ubeta = CALIB_VOLTAGE * sinf(angle);

    float dA, dB, dC;
    SVPWM_Compute(&hsvpwm, Ualpha, Ubeta, &dA, &dB, &dC);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dA * hsvpwm.pwm_period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dB * hsvpwm.pwm_period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dC * hsvpwm.pwm_period);

    HAL_Delay(CALIB_DELAY_MS);
    mech_rev = Encoder_GetAngleRad();
  }
  // tính góc 0
  float mech_avg = 0.5f * (mech_fwd + mech_rev);
  elec_offset = -POLE_PAIRS * mech_avg;
  while (elec_offset < 0)
    elec_offset += 2.0f * M_PI;
  while (elec_offset >= 2.0f * M_PI)
    elec_offset -= 2.0f * M_PI;
  CDC_Log("elec_0 = %.2f", elec_offset);
  DRV8301_DisableGate(&hdrv8301);
}
float target = 0.0;
void Motor_Velocity_OpenLoop_Encoder(float Uq_open)
{
  /* 1. đọc góc cơ */
  float theta_mech = Encoder_GetAngleRad();

  /* 2. tính góc điện */
  float theta_elec = POLE_PAIRS * theta_mech + elec_offset;

  /* wrap */
  while (theta_elec >= 2.0f * M_PI)
    theta_elec -= 2.0f * M_PI;
  while (theta_elec < 0)
    theta_elec += 2.0f * M_PI;

  /* 3. điện áp q-axis cố định */
  float Ualpha = -Uq_open * sinf(theta_elec);
  float Ubeta = Uq_open * cosf(theta_elec);

  /* 4. SVPWM */
  float dA, dB, dC;
  SVPWM_Compute(&hsvpwm, Ualpha, Ubeta, &dA, &dB, &dC);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dA * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dB * hsvpwm.pwm_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dC * hsvpwm.pwm_period);
  if (HAL_GetTick() - last_time > 30)
  {
    last_time = HAL_GetTick();
    CDC_Log("Ua=%f,Ub=%f,da=%f,db=%f,dc=%f", Ualpha, Ubeta, dA, dB, dC);
  }

  // HAL_Delay(1);
}
void print(char *c)
{

  CDC_Transmit_FS((uint8_t *)c, 20);
}

int parse_float_dot(const char *s, float *out)
{
  int sign = 1;
  int ipart = 0;
  int dpart = 0;
  float dscale = 1.0f;
  int has_digit = 0;

  /* skip space */
  while (*s == ' ' || *s == '\r' || *s == '\n')
    s++;

  /* sign */
  if (*s == '-')
  {
    sign = -1;
    s++;
  }
  else if (*s == '+')
  {
    s++;
  }

  /* integer part */
  while (isdigit((unsigned char)*s))
  {
    ipart = ipart * 10 + (*s - '0');
    has_digit = 1;
    s++;
  }

  /* decimal part */
  if (*s == '.')
  {
    s++;
    while (isdigit((unsigned char)*s))
    {
      dpart = dpart * 10 + (*s - '0');
      dscale *= 0.1f;
      has_digit = 1;
      s++;
    }
  }

  if (!has_digit)
    return 0; // parse fail

  *out = sign * (ipart + dpart * dscale);
  return 1; // parse OK
}
// float angle = 0.0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_MOE_ENABLE(&htim1);

  DRV8301_EnableGate(&hdrv8301);
  HAL_Delay(1);
  SVPWM_Init(&hsvpwm, 12.0f, 1000);
  AS5600_Init(&as5600, &hi2c1);
  // HAL_Delay(5000);
  // elec_offset = 3.16;
  // Motor_Calib_EncoderOffset_Bidir();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    angle = AS5600_GetAngleDeg(&as5600);
    CDC_Log("angle = %.2f", AS5600_GetAngleDeg(&as5600));
    if (cdc_rx_flag)
    {
      cdc_rx_flag = 0;
      cdc_rx_buf[cdc_rx_len] = 0;
      if (Command_Parse((char *)cdc_rx_buf, &cmd))
      {
        CDC_Log("CMD=%s KEY=%s VAL=%.3f",
                cmd.cmd,
                cmd.has_key ? cmd.key : "-",
                cmd.has_value ? cmd.value : 0.0f);

        if (strcmp(cmd.cmd, "CALIB") == 0)
        {
          Motor_Calib_EncoderOffset_Bidir();
        }
        else if (strcmp(cmd.cmd, "SET") == 0 && cmd.has_key)
        {
          if (strcmp(cmd.key, "UQ") == 0 && cmd.has_value)
            target = cmd.value;
          else if (strcmp(cmd.key, "SPEED") == 0 && cmd.has_value)
            speed_ref = cmd.value;
        }
        else if (strcmp(cmd.cmd, "EN") == 0)
        {
          DRV8301_EnableGate(&hdrv8301);
        }
        else if (strcmp(cmd.cmd, "DIS") == 0)
        {
          DRV8301_DisableGate(&hdrv8301);
        }
      }
    }
    // Motor_Velocity_OpenLoop_Encoder(target);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 1000 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 150;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2 | GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM14 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#ifndef __SVPWM_H
#define __SVPWM_H

#include <stdint.h>

/* ================= CONFIG ================= */

typedef struct
{
    float Udc;        // DC bus voltage (V)
    float pwm_period; // ARR của timer
} SVPWM_Handle_t;

/* ================= API ==================== */

/* Init */
void SVPWM_Init(SVPWM_Handle_t *hsvpwm, float Udc, float pwm_period);

/* Core SVPWM
 * Input:  Ualpha, Ubeta (điện áp alpha-beta)
 * Output: dutyA/B/C (0.0 -> 1.0)
 */
void SVPWM_Compute(
    SVPWM_Handle_t *hsvpwm,
    float Ualpha,
    float Ubeta,
    float *dutyA,
    float *dutyB,
    float *dutyC
);

#endif

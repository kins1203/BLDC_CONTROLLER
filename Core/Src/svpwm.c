#include "svpwm.h"
#include <math.h>

/* ================= INIT ================= */

void SVPWM_Init(SVPWM_Handle_t *hsvpwm, float Udc, float pwm_period)
{
    hsvpwm->Udc = Udc;
    hsvpwm->pwm_period = pwm_period;
}

/* ================= CORE ================= */

void SVPWM_Compute(
    SVPWM_Handle_t *hsvpwm,
    float Ualpha,
    float Ubeta,
    float *dutyA,
    float *dutyB,
    float *dutyC
)
{
    /* 1. Tính điện áp 3 pha từ alpha-beta */
    float Ua = Ualpha;
    float Ub = -0.5f * Ualpha + 0.8660254f * Ubeta;
    float Uc = -0.5f * Ualpha - 0.8660254f * Ubeta;

    /* 2. Chuẩn hóa theo DC bus */
    float Umax = fmaxf(fmaxf(Ua, Ub), Uc);
    float Umin = fminf(fminf(Ua, Ub), Uc);

    /* 3. SVPWM offset (vector 0) */
    float Uoffset = 0.5f * (Umax + Umin);

    Ua -= Uoffset;
    Ub -= Uoffset;
    Uc -= Uoffset;

    /* 4. Convert sang duty (0 → 1) */
    *dutyA = 0.5f + Ua / hsvpwm->Udc;
    *dutyB = 0.5f + Ub / hsvpwm->Udc;
    *dutyC = 0.5f + Uc / hsvpwm->Udc;

    /* 5. Clamp an toàn */
    if (*dutyA < 0) *dutyA = 0;
    if (*dutyA > 1) *dutyA = 1;

    if (*dutyB < 0) *dutyB = 0;
    if (*dutyB > 1) *dutyB = 1;

    if (*dutyC < 0) *dutyC = 0;
    if (*dutyC > 1) *dutyC = 1;
}

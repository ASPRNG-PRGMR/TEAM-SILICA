/**
 * motor.c - TB6612FNG motor driver layer.
 *
 * SAFETY DESIGN: a "lock" flag is set by Motor_EmergencyKill() (called from the
 * EXTI ISR). While locked, Motor_SetPWM/Forward/Enable refuse to do anything,
 * so even if the main loop was halfway through a "start motor" sequence when
 * the emergency fired, it cannot re-enable the motor afterwards.
 * The lock is only cleared by Motor_Unlock() on UART RESET.
 */
#include "motor.h"
#include "ecu_config.h"

static volatile uint8_t s_locked  = 0;
static volatile uint8_t s_pwm_pct = 0;

/* Small helper: run a block with interrupts disabled, preserving caller state. */
#define CRITICAL_ENTER()  uint32_t _primask = __get_PRIMASK(); __disable_irq()
#define CRITICAL_EXIT()   do { if (!_primask) { __enable_irq(); } } while (0)

void Motor_Init(void)
{
    s_locked = 0;
    __HAL_TIM_SET_COMPARE(ECU_PWM_TIM, ECU_PWM_CHANNEL, 0);   /* duty 0% before start */
    Motor_Stop();                                             /* all driver pins safe  */
    HAL_TIM_PWM_Start(ECU_PWM_TIM, ECU_PWM_CHANNEL);          /* A6 = PWMA @ 0%        */
}

void Motor_SetPWM(uint8_t percent)
{
    CRITICAL_ENTER();
    if (s_locked || percent == 0U) {
        s_pwm_pct = 0;
        __HAL_TIM_SET_COMPARE(ECU_PWM_TIM, ECU_PWM_CHANNEL, 0);
    } else {
        if (percent > 100U) { percent = 100U; }
        uint32_t period = (uint32_t)__HAL_TIM_GET_AUTORELOAD(ECU_PWM_TIM) + 1U;
        s_pwm_pct = percent;
        /* CCR == ARR+1 gives a true 100% duty cycle */
        __HAL_TIM_SET_COMPARE(ECU_PWM_TIM, ECU_PWM_CHANNEL, (period * percent) / 100U);
    }
    CRITICAL_EXIT();
}

void Motor_Forward(void)
{
    CRITICAL_ENTER();
    if (!s_locked) {
        HAL_GPIO_WritePin(ECU_AIN2_PORT, ECU_AIN2_PIN, GPIO_PIN_RESET);  /* B0 = LOW  */
        HAL_GPIO_WritePin(ECU_AIN1_PORT, ECU_AIN1_PIN, GPIO_PIN_SET);    /* A7 = HIGH */
    }
    CRITICAL_EXIT();
}

void Motor_Enable(void)
{
    CRITICAL_ENTER();
    if (!s_locked) {
        HAL_GPIO_WritePin(ECU_STBY_PORT, ECU_STBY_PIN, GPIO_PIN_SET);    /* B1 = HIGH */
    }
    CRITICAL_EXIT();
}

void Motor_Disable(void)
{
    HAL_GPIO_WritePin(ECU_STBY_PORT, ECU_STBY_PIN, GPIO_PIN_RESET);
}

/* Always allowed, even while locked. PWM first, then driver pins. */
void Motor_Stop(void)
{
    s_pwm_pct = 0;
    __HAL_TIM_SET_COMPARE(ECU_PWM_TIM, ECU_PWM_CHANNEL, 0);
    HAL_GPIO_WritePin(ECU_AIN1_PORT, ECU_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ECU_AIN2_PORT, ECU_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ECU_STBY_PORT, ECU_STBY_PIN, GPIO_PIN_RESET);
}

/* Called from the EXTI ISR: only a handful of register writes, no waiting. */
void Motor_EmergencyKill(void)
{
    s_locked = 1;           /* block any in-flight start sequence in main loop */
    Motor_Stop();
}

void Motor_Unlock(void)  { s_locked = 0; }
bool Motor_IsLocked(void){ return s_locked != 0U; }

bool Motor_IsOn(void)
{
    return (s_pwm_pct > 0U) &&
           (HAL_GPIO_ReadPin(ECU_STBY_PORT, ECU_STBY_PIN) == GPIO_PIN_SET);
}

uint8_t Motor_GetPWM(void) { return s_pwm_pct; }

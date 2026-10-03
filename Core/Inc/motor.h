/**
 * motor.h - TB6612FNG motor driver layer (hardware only, no FSM logic).
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

void Motor_Init(void);                  /* outputs safe, PWM running at 0% */
void Motor_SetPWM(uint8_t percent);     /* 0..100 %, ignored (forced 0) while locked */
void Motor_Forward(void);               /* AIN1=HIGH, AIN2=LOW */
void Motor_Enable(void);                /* STBY=HIGH */
void Motor_Disable(void);               /* STBY=LOW  */
void Motor_Stop(void);                  /* PWM=0, AIN1/AIN2=LOW, STBY=LOW */
bool Motor_IsOn(void);                  /* true if driver enabled and PWM > 0 */
uint8_t Motor_GetPWM(void);

/* Emergency support. */
void Motor_EmergencyKill(void);         /* ISR-safe: stop everything and LOCK the motor */
void Motor_Unlock(void);                /* release the lock (motor stays OFF) */
bool Motor_IsLocked(void);

#endif /* MOTOR_H */

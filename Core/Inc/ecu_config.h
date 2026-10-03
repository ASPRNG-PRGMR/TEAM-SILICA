/**
 * ecu_config.h - Pin map and tunable constants for the Mini Motor ECU.
 * Target: STM32F411CEU6, pin names are the CubeMX names (A0, A6, B5, ...).
 * Everything hardware-specific lives here so it is easy to find.
 */
#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#include "main.h"   /* HAL + CubeMX-generated headers */

/* ---- CubeMX-generated peripheral handles (defined in main.c) ---- */
extern ADC_HandleTypeDef  hadc1;   /* ADC1_IN0 on A0              */
extern TIM_HandleTypeDef  htim3;   /* TIM3_CH1 on A6, ~20 kHz PWM */
extern UART_HandleTypeDef huart1;  /* USART1 A9=TX, A10=RX, 115200 8N1 */

#define ECU_PWM_TIM            (&htim3)
#define ECU_PWM_CHANNEL        TIM_CHANNEL_1        /* A6 = PWMA */
#define ECU_ADC                (&hadc1)             /* A0 = ADC1_IN0 */
#define ECU_UART               (&huart1)

/* ---- TB6612FNG control pins ---- */
#define ECU_AIN1_PORT          GPIOA                /* A7 = AIN1 */
#define ECU_AIN1_PIN           GPIO_PIN_7
#define ECU_AIN2_PORT          GPIOB                /* B0 = AIN2 */
#define ECU_AIN2_PIN           GPIO_PIN_0
#define ECU_STBY_PORT          GPIOB                /* B1 = STBY */
#define ECU_STBY_PIN           GPIO_PIN_1

/* ---- Buttons (both active-LOW, internal pull-up) ---- */
#define ECU_START_PORT         GPIOB                /* B5 = START (GPIO input) */
#define ECU_START_PIN          GPIO_PIN_5
#define ECU_ESTOP_PORT         GPIOB                /* B6 = EMERGENCY STOP (EXTI6, falling) */
#define ECU_ESTOP_PIN          GPIO_PIN_6

/* ---- TEMPORARY DIAGNOSTIC BUTTON (remove after testing) ----
 * PA15, GPIO input with internal pull-up, active-LOW. Configured by the user. */
#define ECU_DIAG_PORT          GPIOA                /* A15 = DIAG force-run button */
#define ECU_DIAG_PIN           GPIO_PIN_15

/* ---- State LEDs (active-HIGH) ---- */
#define ECU_LED_IDLE_PORT      GPIOB                /* B8  */
#define ECU_LED_IDLE_PIN       GPIO_PIN_8
#define ECU_LED_READY_PORT     GPIOB                /* B9  */
#define ECU_LED_READY_PIN      GPIO_PIN_9
#define ECU_LED_RUNNING_PORT   GPIOB                /* B10 */
#define ECU_LED_RUNNING_PIN    GPIO_PIN_10
#define ECU_LED_EMERG_PORT     GPIOB                /* B12 */
#define ECU_LED_EMERG_PIN      GPIO_PIN_12

/* ---- Timing / scaling constants ---- */
#define ECU_ADC_MAX            4095U     /* 12-bit */
#define ECU_ADC_PERIOD_MS      10U       /* potentiometer sampling period */
#define ECU_ADC_OVERSAMPLE     4U        /* conversions averaged per sample */
#define ECU_BUTTON_DEBOUNCE_MS 30U
#define ECU_CMD_MAX_LEN        15U       /* longest UART command: "STATUS"/"RESET" */
#define ECU_CMD_IDLE_TIMEOUT_MS 50U      /* terminate a command if the terminal sends no CR/LF */
#define ECU_UART_TX_TIMEOUT_MS 100U

#endif /* ECU_CONFIG_H */

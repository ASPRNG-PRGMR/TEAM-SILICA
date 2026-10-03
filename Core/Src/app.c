/**
 * app.c - Mini Motor ECU: explicit FSM + inputs + UART commands + LEDs.
 *
 *   IDLE --START(B5)--> READY --UART F (PWM>0)--> RUNNING
 *   RUNNING --UART S or PWM==0--> READY
 *   ANY --EMERGENCY STOP (B6 EXTI, falling)--> EMERGENCY --UART RESET--> IDLE
 *
 * Concurrency model: the ONLY interrupt that changes FSM state is the E-stop
 * EXTI. All main-loop state changes go through TryTransition()/StartMotor(),
 * which do a check-and-set with interrupts briefly disabled, so an emergency
 * can never be overwritten by a transition that was in progress.
 */
#include "app.h"
#include "motor.h"
#include "ecu_config.h"
#include <string.h>

/* ------------------------------------------------------------------ */
/* State                                                               */
/* ------------------------------------------------------------------ */
static volatile SystemState g_state       = STATE_IDLE;
static volatile uint8_t     g_estop_event = 0;   /* set in ISR, reported by main loop */

static uint16_t g_adc_raw   = 0;                 /* last averaged ADC value (0..4095) */
static uint8_t  g_cmd_pwm   = 0;                 /* commanded PWM % from potentiometer */
static uint32_t g_adc_tick  = 0;

/* START button debounce */
static uint8_t  g_btn_stable, g_btn_last;
static uint32_t g_btn_tick;

/* UART RX */
#define RX_BUF_SIZE 64U                          /* power of two */
static uint8_t           g_rx_byte;
static volatile uint8_t  g_rx_buf[RX_BUF_SIZE];
static volatile uint8_t  g_rx_head = 0, g_rx_tail = 0;
static char     g_cmd[ECU_CMD_MAX_LEN + 1U];
static uint8_t  g_cmd_len = 0, g_cmd_overflow = 0;
static uint32_t g_cmd_tick = 0;

/* ------------------------------------------------------------------ */
/* Small helpers                                                       */
/* ------------------------------------------------------------------ */
static void UART_Print(const char *s)
{
    HAL_UART_Transmit(ECU_UART, (uint8_t *)s, (uint16_t)strlen(s), ECU_UART_TX_TIMEOUT_MS);
}

static void UART_PrintUInt(uint32_t v)
{
    char buf[11]; int i = 10; buf[i] = '\0';
    do { buf[--i] = (char)('0' + (v % 10U)); v /= 10U; } while (v != 0U);
    UART_Print(&buf[i]);
}

static const char *StateName(SystemState s)
{
    switch (s) {
        case STATE_IDLE:      return "IDLE";
        case STATE_READY:     return "READY";
        case STATE_RUNNING:   return "RUNNING";
        default:              return "EMERGENCY";
    }
}

/* ------------------------------------------------------------------ */
/* LEDs - single place that maps FSM state to the 4 indicator LEDs      */
/* ------------------------------------------------------------------ */
static void LED_Update(SystemState state)
{
    HAL_GPIO_WritePin(ECU_LED_IDLE_PORT,    ECU_LED_IDLE_PIN,    state == STATE_IDLE      ? GPIO_PIN_SET : GPIO_PIN_RESET); /* B8  */
    HAL_GPIO_WritePin(ECU_LED_READY_PORT,   ECU_LED_READY_PIN,   state == STATE_READY     ? GPIO_PIN_SET : GPIO_PIN_RESET); /* B9  */
    HAL_GPIO_WritePin(ECU_LED_RUNNING_PORT, ECU_LED_RUNNING_PIN, state == STATE_RUNNING   ? GPIO_PIN_SET : GPIO_PIN_RESET); /* B10 */
    HAL_GPIO_WritePin(ECU_LED_EMERG_PORT,   ECU_LED_EMERG_PIN,   state == STATE_EMERGENCY ? GPIO_PIN_SET : GPIO_PIN_RESET); /* B12 */
}

/* ------------------------------------------------------------------ */
/* ADC / potentiometer (A0 = ADC1_IN0)                                  */
/* ------------------------------------------------------------------ */
static uint16_t ADC_Read(void)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < ECU_ADC_OVERSAMPLE; i++) {
        HAL_ADC_Start(ECU_ADC);
        if (HAL_ADC_PollForConversion(ECU_ADC, 2) != HAL_OK) {
            HAL_ADC_Stop(ECU_ADC);
            return g_adc_raw;                       /* keep previous value on error */
        }
        sum += HAL_ADC_GetValue(ECU_ADC);
        HAL_ADC_Stop(ECU_ADC);
    }
    return (uint16_t)(sum / ECU_ADC_OVERSAMPLE);
}

/* Linear map 0..4095 -> 0..100 % (1024 -> 25, 2048 -> 50, 3072 -> 75, 4095 -> 100) */
static uint8_t ADC_To_PWM(uint16_t raw)
{
    if (raw > ECU_ADC_MAX) { raw = ECU_ADC_MAX; }
    return (uint8_t)(((uint32_t)raw * 100U) / ECU_ADC_MAX);
}

/* The potentiometer ONLY updates the command value. It never touches the motor
 * directly - the FSM decides whether the command is applied. */
static void ADC_Process(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - g_adc_tick) >= ECU_ADC_PERIOD_MS) {
        g_adc_tick = now;
        g_adc_raw  = ADC_Read();
        g_cmd_pwm  = ADC_To_PWM(g_adc_raw);
    }
}

/* ------------------------------------------------------------------ */
/* START button (B5, active-LOW) - returns true once per debounced press */
/* ------------------------------------------------------------------ */
static void Button_Init(void)
{
    g_btn_stable = g_btn_last =
        (HAL_GPIO_ReadPin(ECU_START_PORT, ECU_START_PIN) == GPIO_PIN_SET);
    g_btn_tick = HAL_GetTick();
}

static bool Button_StartPressed(void)
{
    uint8_t  raw = (HAL_GPIO_ReadPin(ECU_START_PORT, ECU_START_PIN) == GPIO_PIN_SET);
    uint32_t now = HAL_GetTick();

    if (raw != g_btn_last) {                       /* input changed: restart timer */
        g_btn_last = raw;
        g_btn_tick = now;
    } else if ((raw != g_btn_stable) && ((now - g_btn_tick) >= ECU_BUTTON_DEBOUNCE_MS)) {
        g_btn_stable = raw;
        if (raw == 0U) { return true; }            /* stable HIGH->LOW = a press */
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* FSM transition helpers (interrupt-safe check-and-set)               */
/* ------------------------------------------------------------------ */
static bool TryTransition(SystemState from, SystemState to)
{
    bool ok = false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (g_state == from) { g_state = to; ok = true; }
    if (!primask) { __enable_irq(); }
    return ok;
}

/* READY -> RUNNING. All conditions are checked and the motor is started with
 * interrupts disabled for a few register writes, so an E-stop cannot slip in
 * between "check" and "start". */
static bool StartMotor(void)
{
    bool ok = false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if ((g_state == STATE_READY) && (g_cmd_pwm > 0U) && !Motor_IsLocked()) {
        Motor_Forward();                 /* AIN1=HIGH, AIN2=LOW */
        Motor_Enable();                  /* STBY=HIGH           */
        Motor_SetPWM(g_cmd_pwm);
        g_state = STATE_RUNNING;
        ok = true;
    }
    if (!primask) { __enable_irq(); }
    return ok;
}

/* ------------------------------------------------------------------ */
/* FSM_Update - runs every main-loop pass                              */
/* ------------------------------------------------------------------ */
static void FSM_Update(bool start_pressed)
{
    switch (g_state) {

    case STATE_IDLE:
        Motor_Stop();                                        /* motor always OFF */
        if (start_pressed && TryTransition(STATE_IDLE, STATE_READY)) {
            UART_Print("[FSM] IDLE -> READY\r\n");
        }
        break;

    case STATE_READY:
        Motor_Stop();                    /* READY never drives the motor, whatever the pot says */
        break;

    case STATE_RUNNING:
        if (g_cmd_pwm == 0U) {           /* PWM dropped to 0 -> back to READY */
            Motor_Stop();
            if (TryTransition(STATE_RUNNING, STATE_READY)) {
                UART_Print("[FSM] RUNNING -> READY (PWM = 0)\r\n");
            }
        } else {
            Motor_SetPWM(g_cmd_pwm);     /* follow potentiometer (ignored if locked) */
        }
        break;

    case STATE_EMERGENCY:
    default:
        Motor_Stop();                    /* belt and braces; ISR already did this */
        break;
    }
}

/* ------------------------------------------------------------------ */
/* UART commands: F, S, STATUS, RESET                                   */
/* ------------------------------------------------------------------ */
static void PrintStatus(void)
{
    SystemState s = g_state;
    UART_Print("\r\n------------------------\r\n"
               " MINI ECU STATUS\r\n"
               "------------------------\r\n");
    UART_Print("STATE      : ");  UART_Print(StateName(s));                       UART_Print("\r\n");
    UART_Print("SYSTEM     : ");  UART_Print((s == STATE_READY || s == STATE_RUNNING) ? "ENABLED" : "DISABLED");
    UART_Print("\r\n");
    UART_Print("MOTOR      : ");  UART_Print(Motor_IsOn() ? "ON" : "OFF");       UART_Print("\r\n");
    UART_Print("PWM        : ");  UART_PrintUInt(Motor_GetPWM());                UART_Print("%\r\n");
    UART_Print("ADC VALUE  : ");  UART_PrintUInt(g_adc_raw);                     UART_Print("\r\n");
    UART_Print("EMERGENCY  : ");  UART_Print(s == STATE_EMERGENCY ? "YES" : "NO"); UART_Print("\r\n");
    UART_Print("------------------------\r\n");
}

static void Cmd_F(void)
{
    switch (g_state) {
    case STATE_READY:
        if (StartMotor()) {
            UART_Print("[FSM] READY -> RUNNING\r\n");
        } else if (g_cmd_pwm == 0U) {
            UART_Print("F ignored: PWM is 0, motor stays OFF\r\n");
        } else {
            UART_Print("F rejected\r\n");
        }
        break;
    case STATE_IDLE:      UART_Print("F rejected: system not enabled (press START)\r\n"); break;
    case STATE_EMERGENCY: UART_Print("F rejected: EMERGENCY active (send RESET)\r\n");    break;
    default:              UART_Print("Already RUNNING\r\n");                               break;
    }
}

static void Cmd_S(void)
{
    Motor_Stop();                                    /* S ALWAYS stops the motor */
    if (TryTransition(STATE_RUNNING, STATE_READY)) {
        UART_Print("[FSM] RUNNING -> READY (S)\r\n");
    } else {
        UART_Print("Motor stopped\r\n");
    }
}

static void Cmd_Reset(void)
{
    if (g_state != STATE_EMERGENCY) {
        UART_Print("RESET ignored: only valid in EMERGENCY\r\n");
        return;
    }

    bool ok = false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* Refuse to leave EMERGENCY while the E-stop button is still pressed. */
    if (HAL_GPIO_ReadPin(ECU_ESTOP_PORT, ECU_ESTOP_PIN) == GPIO_PIN_SET) {
        Motor_Stop();                                /* PWM = 0, motor OFF */
        Motor_Unlock();                              /* motor stays OFF; just allows later start */
        g_state = STATE_IDLE;                        /* NOT ready: START must be pressed again */
        ok = true;
    }
    if (!primask) { __enable_irq(); }

    UART_Print(ok ? "[FSM] EMERGENCY -> IDLE (press START)\r\n"
                  : "RESET rejected: release the E-STOP button first\r\n");
}

static void UART_ProcessCommand(const char *cmd)
{
    if      (strcmp(cmd, "F")      == 0) { Cmd_F(); }
    else if (strcmp(cmd, "S")      == 0) { Cmd_S(); }
    else if (strcmp(cmd, "STATUS") == 0) { PrintStatus(); }
    else if (strcmp(cmd, "RESET")  == 0) { Cmd_Reset(); }
    else                                 { UART_Print("ERR: unknown command (F, S, STATUS, RESET)\r\n"); }
}

/* Collect bytes from the RX ring buffer into a command line.
 * A command ends at CR/LF, or after ECU_CMD_IDLE_TIMEOUT_MS of silence
 * (so terminals that send no line ending still work). Case-insensitive. */
static void UART_Poll(void)
{
    uint32_t now = HAL_GetTick();

    while (g_rx_tail != g_rx_head) {
        uint8_t c = g_rx_buf[g_rx_tail];
        g_rx_tail = (uint8_t)((g_rx_tail + 1U) & (RX_BUF_SIZE - 1U));
        g_cmd_tick = now;

        if (c == '\r' || c == '\n') {
            if (g_cmd_len > 0U || g_cmd_overflow) {
                g_cmd[g_cmd_len] = '\0';
                if (g_cmd_overflow) { UART_Print("ERR: command too long\r\n"); }
                else                { UART_ProcessCommand(g_cmd); }
                g_cmd_len = 0; g_cmd_overflow = 0;
            }
        } else if (c > ' ') {                         /* skip spaces/control chars */
            if (c >= 'a' && c <= 'z') { c = (uint8_t)(c - 'a' + 'A'); }
            if (g_cmd_len < ECU_CMD_MAX_LEN) { g_cmd[g_cmd_len++] = (char)c; }
            else                             { g_cmd_overflow = 1; }
        }
    }

    if ((g_cmd_len > 0U) && ((now - g_cmd_tick) >= ECU_CMD_IDLE_TIMEOUT_MS)) {
        g_cmd[g_cmd_len] = '\0';
        if (g_cmd_overflow) { UART_Print("ERR: command too long\r\n"); }
        else                { UART_ProcessCommand(g_cmd); }
        g_cmd_len = 0; g_cmd_overflow = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */
void App_Init(void)
{
    Motor_Init();                                    /* motor OFF, PWM 0% */
    g_state = STATE_IDLE;
    LED_Update(STATE_IDLE);

    /* If the E-stop is already held at power-up, start in EMERGENCY (fail-safe). */
    if (HAL_GPIO_ReadPin(ECU_ESTOP_PORT, ECU_ESTOP_PIN) == GPIO_PIN_RESET) {
        Motor_EmergencyKill();
        g_state = STATE_EMERGENCY;
        LED_Update(STATE_EMERGENCY);
    }

    Button_Init();
    g_adc_raw = ADC_Read();
    g_cmd_pwm = ADC_To_PWM(g_adc_raw);
    g_adc_tick = HAL_GetTick();

    HAL_UART_Receive_IT(ECU_UART, &g_rx_byte, 1);    /* start interrupt-driven RX */
    UART_Print("\r\nMINI ECU ready. Commands: F, S, STATUS, RESET\r\n");
    UART_Print(g_state == STATE_EMERGENCY ? "[FSM] Power-up in EMERGENCY (E-STOP held)\r\n"
                                          : "[FSM] State: IDLE (press START)\r\n");
}

/* TEMPORARY DIAGNOSTIC: while PA15 is LOW, push the FSM through its normal
 * path (IDLE -> READY -> RUNNING) using the existing TryTransition() and
 * StartMotor(). All existing safety checks still apply (E-stop/EMERGENCY lock,
 * PWM > 0). Does nothing in EMERGENCY. Remove after testing. */
static void Diag_ForceRunning(void)
{
    if (HAL_GPIO_ReadPin(ECU_DIAG_PORT, ECU_DIAG_PIN) != GPIO_PIN_RESET) {
        return;                                      /* PA15 HIGH: normal behaviour */
    }
    if (g_state == STATE_IDLE) {
        TryTransition(STATE_IDLE, STATE_READY);      /* same as the START button */
    }
    if (g_state == STATE_READY) {
        StartMotor();                                /* same as UART "F" */
    }
}

void App_Run(void)
{
    static SystemState displayed = (SystemState)0xFF;

    ADC_Process();                                   /* 1,2: ADC -> commanded PWM */
    bool start_pressed = Button_StartPressed();      /* 3: START button           */
    UART_Poll();                                     /* 4: UART commands          */
    Diag_ForceRunning();                             /* TEMP: PA15 diagnostic     */
    FSM_Update(start_pressed);                       /* 5,6: transitions + motor  */

    if (g_estop_event) {                             /* report E-stop (non-critical work) */
        g_estop_event = 0;
        UART_Print("\r\n[FSM] !!! EMERGENCY STOP !!! (send RESET after releasing button)\r\n");
    }

    SystemState s = g_state;                         /* 7: LEDs always mirror the FSM */
    if (s != displayed) {
        displayed = s;
        LED_Update(s);
    }
}

/* ------------------------------------------------------------------ */
/* Interrupt callbacks                                                 */
/* ------------------------------------------------------------------ */

/* EMERGENCY STOP: B6 = EXTI6, falling edge. Runs from EXTI9_5_IRQHandler via HAL.
 * Kept minimal and deterministic: kill the motor, latch the state, set a flag. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ECU_ESTOP_PIN) {
        Motor_EmergencyKill();                       /* PWM=0, AIN1/AIN2/STBY LOW, motor locked */
        if (g_state != STATE_EMERGENCY) {
            g_state = STATE_EMERGENCY;
            g_estop_event = 1;                       /* main loop prints the message */
        }
    }
}

/* USART1 RX byte received */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint8_t next = (uint8_t)((g_rx_head + 1U) & (RX_BUF_SIZE - 1U));
        if (next != g_rx_tail) {                     /* drop byte if buffer full */
            g_rx_buf[g_rx_head] = g_rx_byte;
            g_rx_head = next;
        }
        HAL_UART_Receive_IT(huart, &g_rx_byte, 1);   /* re-arm */
    }
}

/* Overrun/framing error: just re-arm reception */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(huart, &g_rx_byte, 1);
    }
}

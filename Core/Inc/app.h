/**
 * app.h - Mini Motor ECU application (FSM, inputs, UART commands, LEDs).
 */
#ifndef APP_H
#define APP_H

typedef enum
{
    STATE_IDLE,
    STATE_READY,
    STATE_RUNNING,
    STATE_EMERGENCY
} SystemState;

void App_Init(void);   /* call once, after all MX_xxx_Init() */
void App_Run(void);    /* call repeatedly from while(1) */

#endif /* APP_H */

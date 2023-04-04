/* Copyright (C) 2023 Michael Fitzgerald */

#define DEBUG_MODULE "OpenMV"

#include "openmv.h"

#include "debug.h"
#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "deck.h"
#include "param.h"
#include "commander.h"
#include "uart1.h"
#include "motors.h"

#include <math.h>

//#define OMV_PIN DECK_GPIO_IO1
#define OMV_PIN_IN UART1_GPIO_RX_PIN
#define OMV_PIN_OUT UART1_GPIO_TX_PIN

openmv_state_t openmv_state;

static bool isInit = false;
static xTimerHandle timer;

static void omvSetState(uint8_t target_x_byte,
                        uint8_t target_y_byte,
                        uint8_t target_z_byte)
{
    memcpy(&openmv_state.target_x, &target_x_byte, 1);
    memcpy(&openmv_state.target_y, &target_y_byte, 1);
    memcpy(&openmv_state.target_z, &target_z_byte, 1);
    DEBUG_PRINT("OpenMV target: (%d, %d, %d)\n",
                (int)openmv_state.target_x,
                (int)openmv_state.target_y,
                (unsigned int)openmv_state.target_z);
}

static void omvTimer(xTimerHandle timer)
{
    //int status = digitalRead(OMV_PIN);
    
    // get serial data from UART
    // 1. loop until start byte is found
    // 2. get all 64 bytes, including start byte
    uint8_t buf[8];
    buf[0] = 0;
    buf[1] = 0;
    bool res;
    // find the sequence 0x6969, for synchronization
    do {
        do {
            res = uart1GetDataWithTimeout(&buf[0], M2T(100));
            if (!res) {
                DEBUG_PRINT("OMV: connection timed out (signature, first byte)\n");
                return;
            }
        } while (buf[0] != 0x69);
        res = uart1GetDataWithTimeout(&buf[1], M2T(100));
        if (!res) {
            DEBUG_PRINT("OMV: connection timed out (signature, second byte)\n");
            return;
        }
    } while (buf[1] != 0x69);
    // read remaining bytes
    for (int i = 2; i < sizeof(buf); i++) {
        res = uart1GetDataWithTimeout(&buf[i], M2T(100));
        if (!res) {
            DEBUG_PRINT("OMV: connection timed out\n");
            return;
        }
    }

    // parse message
    switch (buf[2]) {
    case 0: // pass
        break;
    case 1: // set target setpoint
        //omvSetMotorSpeed(buf[2]);
        openmv_state.active = buf[3];
        if (buf[3]) {
            omvSetState(buf[4], buf[5], buf[6]);
        } else {
            DEBUG_PRINT("(Skipping...)\n");
        }
        break;
    default:
        break;
    }
    
    return;
}

static void omvInit()
{
    if (isInit) return;
    
    // clear state
    memset(&openmv_state, 0, sizeof(openmv_state_t));

    // initialize UART1 comms
    //pinMode(OMV_PIN, INPUT);
    uart1InitWithParity(115200, uart1ParityNone);
    if (!uart1Test()) {
        DEBUG_PRINT("Could not use UART1\n");
        return;
    }

    timer = xTimerCreate("omvTimer", M2T(100), pdTRUE, NULL, omvTimer);
    xTimerStart(timer, 100);

    isInit = true;
    DEBUG_PRINT("OpenMV connected.\n");
}

static bool omvTest()
{
    DEBUG_PRINT("Mission accomplished!\n");
    return true;
}

static const DeckDriver omvDriver = {
    .vid = 0,
    .pid = 0,
    .name = "openmv",
    .init = omvInit,
    .test = omvTest,
    .usedPeriph = DECK_USING_UART1,
};

DECK_DRIVER(omvDriver);
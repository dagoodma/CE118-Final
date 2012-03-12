/*
 * File:   IRnew.c
 * Author: dagoodma, hahernan, jurjohns
 *
 * Infrared beacon sensor module. Switches between the main and angled
 * beacon sensors to aquire readings from both.
 *
 */

#include <p32xxxx.h>
#include "serial.h"
#include "timers.h"
#include "PORTS.h"
#include "AD.H"
#include "IR_new.h"
#include "LED.h"


//#define USE_LEDS
//#define IR_TEST

//#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
#define dbprintf(...) printf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif
#define TIMER_NUM 4
#define UPDATE_DELAY 1


#define IR_MAIN AD_PORTW6
#define IR_ANGLE AD_PORTW7

#define MAIN_HIGH_THRESHOLD 291
#define MAIN_LOW_THRESHOLD 277
#define ANGLE_HIGH_THRESHOLD 999
#define ANGLE_LOW_THRESHOLD 976


/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/

enum { IR_MAIN_I, IR_ANGLE_I };

static unsigned int irState[] = { 0, 0 };
static unsigned int irThreshold[] = { MAIN_HIGH_THRESHOLD, ANGLE_HIGH_THRESHOLD };

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/
char Is_Angle_Triggered(void);
char Is_Main_Triggered(void);

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/
char IsAngleTriggered() {
    unsigned int val = ReadADPin(IR_ANGLE);
    //dbprintf("\nAngle=%d", val);
    if (val > irThreshold[IR_ANGLE_I]) {
        irThreshold[IR_ANGLE_I] = ANGLE_LOW_THRESHOLD;
        return ON;
    }
    if (val < irThreshold[IR_ANGLE_I]) {
        irThreshold[IR_ANGLE_I] = ANGLE_HIGH_THRESHOLD;
        return OFF;
    }
    return OFF;
}

char IsMainTriggered() {
    unsigned int val = ReadADPin(IR_MAIN);
    //dbprintf("\nMain=%d", val);
    if (val > irThreshold[IR_MAIN_I]) {
        irThreshold[IR_MAIN_I] = MAIN_LOW_THRESHOLD;
        return ON;
    }
    if (val < irThreshold[IR_MAIN_I]) {
        irThreshold[IR_MAIN_I] = MAIN_HIGH_THRESHOLD;
        return OFF;
    }
    return OFF;
}

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/
char IR_Init() {
    //dbprintf("\nInitializing the IR Sensor Module.");

    #ifdef USE_LEDS
    LED_Init(LED_BANK3);
    LED_OffBank(LED_BANK3, 0xF);

    #endif

    //InitTimer(TIMER_NUM, UPDATE_DELAY);

    return SUCCESS;
}

char IR_Update() {
    // Check main sensor
    //if (IsTimerExpired(TIMER_NUM)) {
        char result = IsMainTriggered();
        #ifdef USE_LEDS
        if (result)
            LED_OnBank(LED_BANK3, 0x1);
        else
            LED_OffBank(LED_BANK3, 0x1);
        #endif
        irState[IR_MAIN_I] = result;

        // Check angle sensor
        result = IsAngleTriggered();
        #ifdef USE_LEDS
        if (result)
            LED_OnBank(LED_BANK3, 0x8);
        else
            LED_OffBank(LED_BANK3, 0x8);
        #endif
         irState[IR_ANGLE_I] = result;
         
         //InitTimer(TIMER_NUM, UPDATE_DELAY);
     //}

     return TRUE;
}

char IR_MainTriggered() {
    IR_Update();
    return irState[IR_MAIN_I];
}

char IR_AngleTriggered() {
    IR_Update();
    return irState[IR_ANGLE_I];
}

unsigned int IR_MainReading() {
    return ReadADPin(IR_MAIN);
}

unsigned int IR_AngleReading() {
    return ReadADPin(IR_ANGLE);
}

char IR_End() {
    AD_End();
    return SUCCESS;
}


/*******************************************************************************
 * TEST HARNESS                                                                *
 ******************************************************************************/
#ifdef IR_TEST
#ifndef DEBUG_VERBOSE
#define DEBUG_VERBOSE
#endif

#define NOPCOUNT 990000
#define DELAY() for(i=0;i < NOPCOUNT; i++) __asm("nop")

int main(void) {
    SERIAL_Init();
    TIMERS_Init();
    char i, j = 0;
    int k, l = 0;
    int time = GetTime();
    INTEnableSystemMultiVectoredInt();
    AD_Init(IR_PINS);
    IR_Init();

    while (1) {
        k = ReadADPin(IR_MAIN);
        l = ReadADPin(IR_ANGLE);
		char mainTrig = '_';
		char angleTrig = '_';
		if (IR_MainTriggered())
			mainTrig = 'x';
		if (IR_AngleTriggered())
			angleTrig = 'x';
			
        //if (time > GetTime() + 500) {
            dbprintf("\n %cMain : %d \n %cAngle : %d",mainTrig, k, angleTrig, l);
            //time = GetTime();
        //}
        while (!IsTransmitEmpty()); // bad, this is blocking code
    }
    return 0;
}
#endif

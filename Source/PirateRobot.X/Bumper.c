/*
 * File:   TapeSensor.c
 * Author: dagoodma
 *
 * Taking 3 samples, at 
 *
 */

#include <p32xxxx.h>
#include "serial.h"
#include "timers.h"
#include "PORTS.h"
//#include "LED.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
//#define BUMP_TEST

#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define TIMER_NUM 1
#define UPDATE_DELAY 2 // ms

#define BUMPERCOUNT 3

#define BUMPER_RIGHT_BIT PORTZ03_BIT // _RE4
#define BUMPER_RIGHT_TRIS PORTZ03_TRIS
#define BUMPER_LEFT_BIT PORTZ04_BIT
#define BUMPER_LEFT_TRIS PORTZ04_TRIS
#define BUMPER_CENTER_BIT PORTZ05_BIT
#define BUMPER_CENTER_TRIS PORTZ05_TRIS

#define MAXCOUNT 0x3


// Delay times
#define READDELAY 2 // milisecond

/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/


enum bumperIndex {BUMPER_RIGHT_I, BUMPER_LEFT_I, BUMPER_CENTER_I};
static unsigned char bumperCounter[] = {0, 0, 0};
static unsigned int bumperPort[] = {0, 0, 0};

/**
static unsigned int const ledPortMap[] = {
    LED_BANK1_3,
    LED_BANK2_3,
    LED_BANK3_3,
    LED_BANK2_0, 
    LED_BANK2_2,
    LED_BANK1_2,
    LED_BANK3_2
};
 */



/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/

static void UpdateBumperCounters();
static char ReadBumpers();

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

static void UpdateBumperCounters() {
    int i;
    ReadBumpers();
    // Iterate over bumpers and adjust saturating counters
    for (i = 0; i < BUMPERCOUNT; i++) {
        if (bumperPort[i]) {
            if (bumperCounter[i] < MAXCOUNT)
                bumperCounter[i] += 1;
        }
        else {
            if (bumperCounter[i] > 0)
                bumperCounter[i] -= 1;
        }
    }

}

static char ReadBumpers() {
    bumperPort[BUMPER_RIGHT_I] = !BUMPER_RIGHT_BIT;
    bumperPort[BUMPER_LEFT_I] = !BUMPER_LEFT_BIT;
    bumperPort[BUMPER_CENTER_I] = !BUMPER_CENTER_BIT;
    return SUCCESS;
}

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/
char Bumper_Init() {
    dbprintf("\nInitializing the Bumper Sensor Module.");

    InitTimer(TIMER_NUM, UPDATE_DELAY);

    // Define inputs
    BUMPER_RIGHT_TRIS = 1;
    BUMPER_LEFT_TRIS = 1;
    BUMPER_CENTER_TRIS = 1;

    dbprintf("\nBumper sensors initialized (%d)", BUMPERCOUNT);

    return SUCCESS;

}

char Bumper_Update() {
    if (IsTimerExpired(TIMER_NUM)) {
        UpdateBumperCounters();
        InitTimer(TIMER_NUM, UPDATE_DELAY);
    }
    return SUCCESS;
}

char Bumper_LeftTriggered() {
    return bumperPort[BUMPER_LEFT_I]; // bumperCounter[BUMPER_LEFT_I] > (MAXCOUNT / 2);
}

char Bumper_CenterTriggered() {
    return bumperPort[BUMPER_CENTER_I]; // bumperCounter[BUMPER_CENTER_I] > (MAXCOUNT / 2);
}

char Bumper_RightTriggered() {
    return bumperPort[BUMPER_RIGHT_I]; // bumperCounter[BUMPER_RIGHT_I] > (MAXCOUNT / 2);
}

char Bumper_End() {
    StopTimer(TIMER_NUM);
    return SUCCESS;
}

/*******************************************************************************
 * TEST HARNESS                                                                *
 ******************************************************************************/
#ifdef BUMP_TEST
#ifndef DEBUG_VERBOSE
#define DEBUG_VERBOSE
#endif

#define NOPCOUNT 990000
#define DELAY() for(i=0;i < NOPCOUNT; i++) __asm("nop")

int main(void) {
    SERIAL_Init();
    TIMERS_Init();
    int i = 0;

    INTEnableSystemMultiVectoredInt();

    Bumper_Init();

    // Test routine (above goes in init)
    printf("\nHello tester!");
    DELAY();

    while(1) {
        Bumper_Update();

        printf("\nStates: LEFT=(%x), CENTER=(%x), RIGHT=(%x)",
            Bumper_LeftTriggered(), Bumper_CenterTriggered(),
            Bumper_RightTriggered());
               
            while (!IsTransmitEmpty()); // bad, this is blocking code
    } // end of loop

    Bumper_End();
    return 0;
} // test harness
#endif


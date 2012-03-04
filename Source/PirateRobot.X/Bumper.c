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
#include "pwm.h"
#include "AD.h"
#include "LED.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define USE_LEDS

#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define BUMPERCOUNT 3

#define BUMPER_LEFT_BIT
#define BUMPER_LEFT_TRIS
#define BUMPER_CENTER_BIT
#define BUMPER_CENTER_TRIS
#define BUMPER_RIGHT_BIT
#define BUMPER_RIGHT_TRIS

#define MAXCOUNT = 0x3


// Delay times
#define READDELAY 2 // milisecond

/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/


enum bumperIndex {BUMPER_LEFT_I, BUMPER_CENTER_I, BUMPER_RIGHT_I};
static unsigned char bumperCounter[] = {0, 0, 0};
static unsigned int bumperPortMap[] = {BUMPER_LEFT_BIT, BUMPER_CENTER_BIT, BUMPER_RIGHT_BIT};

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

void UpdateBumperCounters();

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

void UpdateBumperCounters() {
    int i;
    // Iterate over bumpers and adjust saturating counters
    for (i = 0; i < BUMPERCOUNT; i++) {
        if (ReadBumper(i)) {
            if (bumperCounter[i] < MAXCOUNT)
                bumperCounter[i] += 1;
        }
        else {
            if (bumperCounter[i] > 0)
                bumperCounter[i] -= 1;
        }
    }

}

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/
char Bumper_LeftTriggered() {
    return bumperCounter[BUMPER_LEFT_I] > (MAXCOUNT / 2);
}

char Bumper_CenterTriggered() {
    return bumperCounter[BUMPER_CENTER_I] > (MAXCOUNT / 2);
}

char Bumper_RightTriggered() {
    return bumperCounter[BUMPER_RIGHT_I] > (MAXCOUNT / 2);
}

/*******************************************************************************
 * TEST HARNESS                                                                *
 ******************************************************************************/

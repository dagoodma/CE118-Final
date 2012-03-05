/*
 * File:   IR.c
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
//#include "LED.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
//#define IR_TEST

#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define TIMER_NUM 3
#define UPDATE_DELAY 2 // ms

#define SENSORCOUNT 2

#define IR_BIT PORTZ06_BIT
#define IR_TRIS PORTZ06_TRIS
#define IR_MAIN_SELECT
#define IR_MAIN_SELECT_TRIS
#define IR_ANGLE_SELECT
#define IR_ANGLE_SELECT_TRIS

#define COUNTMAX 0x4
#define SHIFTAMOUNT 0x1 // amount to divide by (2)

// Delay times
#define READDELAY 2 // msecs

/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/

enum irIndex {IR_MAIN_I, IR_ANGLE_I};
enum irSelect {OFF, TOGGLE};
static unsigned char irCounter[] = {0, 0}; // counter values
static unsigned int irPort[] = {0, 0}; // IR bit vlaues
static unsigned int irSelectPort[] = {IR_MAIN_SELECT, IR_ANGLE_SELECT};
static unsigned char irState[] = {0, 0};

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

static unsigned char timesRead = 0;
static enum {init,read,wait} irState;
static char mainIsSelected = 0;


/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/

static void UpdateIRCounter();
static void ReadIR();
static void UpdateIRState();
static void ClearIRCounters();
static char SeesIR(int index);
static void ToggleIR();

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * @Function: UpdateIRCounter
 * @remark Updates the currently activated sensor's counter. 1 is added when
 *         the sensor detects an IR signal.
 * @date 2012.3.5 12:13 */
static void UpdateIRCounter() {
    ReadIR();
    if (mainIsSelected) {
        // update main detector counter
        if (irPort[IR_MAIN_I] && irCounter[IR_MAIN_I] < COUNTMAX )
            irCounter[IR_MAIN_I] += 1;
    }
    else {
        // update angle detector counter
        if (irPort[IR_ANGLE_I] && irCounter[IR_ANGLE_I] < COUNTMAX )
            irCounter[IR_ANGLE_I] += 1;
    }
}


/**
 * @Function: ReadIR
 * @remark Reads the current selected IR sensor.
 * @date 2012.3.5 12:13 */
static void ReadIR() {
    if (mainIsSelected)
        irPort[IR_MAIN_I] = IR_MAIN_BIT;
    else
        irPort[IR_ANGLE_I] = IR_ANGLE_BIT;
}


/**
 * @Function: SeesIR
 * @param index, corresponding to a beacon (IR_MAIN_I or IR_ANGLE_I)
 * @returns TRUE or FALSE
 * @remark Whether the specified beacon's (counter - 1)/2 is greater
 *         than zero.
 * @date 2012.3.5 12:13 */
static char SeesIR(unsigned int index) {
    // divide and offset
    if (irCounter[index] > 0) {
        irCounter[index] -= 1;
        irCounter[index] >>= SHIFTAMOUNT;
    }

    return irCounter[index] > 0;
}

/**
 * @Function: UpdateIRState
 * @remark Updates the state of the currently selected beacon using
 *         SeesIR();
 * @date 2012.3.5 12:13 */
static void UpdateIRState() {
    if (mainIsSelected) {
        irState[IR_MAIN_I] = SeesIR(IR_MAIN_I);
    }
    else {
        irState[IR_MAIN_I] = SeesIR(IR_MAIN_I);
    }
}

/**
 * @Function: UpdateIRState
 * @remark Clears both beacon's counters.
 * @date 2012.3.5 12:33 */
static void ClearIRCounters() {
    int i;
    for(i = 0; i < SENSORCOUNT; i++) {
        irCounter[i] = 0;
    }
}

/**
 * @Function: ToggleIR
 * @param select, optionally toggle or disable sensors (default=TOGGLE)
 * @remark Toggles between the main and angle beacons, and activates
 *         the appropriate select port.
 * @date 2012.3.5 12:47 */
static void ToggleIR(unsigned int select=TOGGLE) {
    if (select == TOGGLE)
        mainIsSelected ^= 1; // toggle between
    else
        mainIsSelected = 0;

    
    // iterate through select ports and flip the bits
    int i;
    for (i = 0; i < SENSORCOUNT; i++) {
        if (select == TOGGLE)
            irSelectPort[i] ^= 1;
        else
            irSelectPort[i] = 0;
    }
}

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/
char IR_Init() {
    dbprintf("\nInitializing the IR Sensor Module.");
    irState = init;

    InitTimer(TIMER_NUM, UPDATE_DELAY);

    // Define input/outputs
    IR_TRIS = 1;
    IR_MAIN_SELECT_TRIS = 0;
    IR_ANGLE_SELECT_TRIS = 0;

    // Enable the main beacon
    mainIsSelected = 1;
    irSelectPort[IR_MAIN_I] = 1;
    irState = wait;


    dbprintf("\nIR sensors initialized (%d)", SENSORCOUNT);

    return SUCCESS;

}

char IR_HandleSM() {
    switch (irSate) {
        case read:
            // read a sensor
            UpdateIRCounter();
            
            timesRead++;
            if (timesRead >= COUNTMAX) {
                UpdateIRState();
                // switch beacons
                ClearIRCounters();
                timesRead = 0;
                ToggleIR();
            }
    
            InitTimer(TIMER_NUM, UPDATE_DELAY);
            irState = wait;
            break;

        case wait:
            if (IsTimerExpired(TIMER_NUM)) {
                irState = read;
            }

        case default:
            dbprintf("\nHorrible Error");
            return ERROR;
            break;
    }
    return SUCCESS;
}

char IR_MainTriggered() {
    return irSate[IR_MAIN_I];
}

char IR_AngleTriggered() {
    return irSate[IR_ANGLE_I];
}

char IR_End() {
    StopTimer(TIMER_NUM);
    ToggleIR(OFF);

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
    TIMERS_Init();
    SERIAL_Init();
    int i = 0;
    char mainSelected;
    char angleSelected;

    INTEnableSystemMultiVectoredInt();
    IR_Init();

    while(1) {

        if (mainIsSelected) {
            mainSelected = '*';
            angleSelected = '';
        }
        else {
            mainSelected = '';
            angleSelected = '*';
        }


        printf("\n%cMain: bit=%c count=%c state=%c",mainSelected,
            irPort[IR_MAIN_I], irCounter[IR_MAIN_I], IR_MainTriggered());
        printf("\n%cAngle: bit=%c count=%c state=%c",angleSelected,
            irPort[IR_ANGLE_I], irCounter[IR_ANGLE_I], IR_AngleTriggered());

        while (!IsTransmitEmpty()); // bad, this is blocking code
    } // end of loop


    IR_End();

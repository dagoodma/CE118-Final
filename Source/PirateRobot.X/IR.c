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
#include "IR.h"
//#include "LED.h"
#ifdef IR_ADC
#include "AD.H"
#endif

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
//#define IR_TEST

//#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define TIMER_NUM 3
#define UPDATE_DELAY 12 // ms

#define SENSORCOUNT 2

// ------------------ IR Ports ----------------------
// *********** ADC definition in TapeSensor.h ************
#ifndef IR_ADC
#define IR_READ_BIT PORTW06_BIT
#define IR_READ_TRIS PORTW06_TRIS
#else
// ADC thresholds
#define HIGH_THRESHOLD 0x290
#define LOW_THRESHOLD 0x0d0
#define FLAT_THRESHOLD 0x1f0
#endif

#define IR_MAIN_SELECT PORTX10_LAT
#define IR_MAIN_SELECT_TRIS PORTX10_TRIS
#define IR_ANGLE_SELECT PORTX11_LAT
#define IR_ANGLE_SELECT_TRIS PORTX11_TRIS

#define COUNTMAX 0x8
#define SHIFTAMOUNT 0x3 // amount to divide by (2)


/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/

enum irIndex {IR_MAIN_I, IR_ANGLE_I};
enum irSelect {OFF, ANGLE, MAIN};
static unsigned char irCounter[] = {0, 0}; // counter values
static unsigned int irPort[] = {0, 0}; // Always has last reading
//static unsigned char irSelectPort[] = {IR_MAIN_SELECT, IR_ANGLE_SELECT};
static unsigned char irFound[] = {0, 0};

#ifdef IR_ADC
static unsigned int irReading[] = {0, 0};
static unsigned int irThreshold[] = {LOW_THRESHOLD, LOW_THRESHOLD};
#endif

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
static enum { init, read, waitOn, waitOff} irState = init;
static char mainIsSelected = 0;


/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/

static void UpdateIRCounter();
static void ReadIR();
static void UpdateIRState();
static void ClearIRCounters();
static char SeesIR(unsigned int index);
static void SwitchIR(unsigned int select);

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
#ifndef IR_ADC
        if (irPort[IR_MAIN_I] && irCounter[IR_MAIN_I] < COUNTMAX )
            irCounter[IR_MAIN_I] += 1;
#else
        irReading[IR_MAIN_I] += irPort[IR_MAIN_I]; // ADC value
#endif
    }
    else {
        // update angle detector counter
#ifndef IR_ADC
        if (irPort[IR_ANGLE_I] && irCounter[IR_ANGLE_I] < COUNTMAX )
            irCounter[IR_ANGLE_I] += 1;
#else
        irReading[IR_ANGLE_I] += irPort[IR_ANGLE_I]; // ADC value
#endif
    }
}


/**
 * @Function: ReadIR
 * @remark Reads the current selected IR sensor.
 * @date 2012.3.5 12:13 */
static void ReadIR() {
    //printf("HER!");
#ifndef IR_ADC
    unsigned int reading = IR_READ_BIT;
#else
    unsigned int reading = ReadADPin(IR_READ);
    printf("\nADC: %x", reading);
#endif
    if (mainIsSelected) {
        printf(" -- main");
              
        irPort[IR_MAIN_I] = reading;
    }
    else {
        printf(" -- angle");
        irPort[IR_ANGLE_I] = reading;
    }
    printf(" -- m:%x a:%x", IR_MAIN_SELECT, IR_ANGLE_SELECT);

}


/**
 * @Function: SeesIR
 * @param index, corresponding to a beacon (IR_MAIN_I or IR_ANGLE_I)
 * @returns TRUE or FALSE
 * @remark Whether the specified beacon's (counter - 1)/2 is greater
 *         than zero.
 * @date 2012.3.5 12:13 */
static char SeesIR(unsigned int index) {
#ifndef IR_ADC
    // divide and offset
    if (irCounter[index] > 0) {
        irCounter[index] -= 1;
        irCounter[index] >>= SHIFTAMOUNT;
    }

    return irCounter[index] > 0;
#else
    irReading[index] >>= SHIFTAMOUNT;
    printf("\nReading: %x", irReading[index]);
    //if (irReading[index] >= ON_THRESHOLD) {
    if (irReading[index] >= irThreshold[index]) {
        //irThreshold[index] = LOW_THRESHOLD;
        printf(" -- HIGH");
        return TRUE;
    }
    //irThreshold[index] = HIGH_THRESHOLD;
     printf(" -- LOW\n");
    return FALSE;
#endif

}

/**
 * @Function: UpdateIRState
 * @remark Updates the state of the currently selected beacon using
 *         SeesIR();
 * @date 2012.3.5 12:13 */
static void UpdateIRState() {
    if (mainIsSelected) {
        irFound[IR_MAIN_I] = SeesIR(IR_MAIN_I);
    }
    else {
        irFound[IR_ANGLE_I] = SeesIR(IR_ANGLE_I);
    }
}

/**
 * @Function: UpdateIRState
 * @remark Clears both beacon's counters.
 * @date 2012.3.5 12:33 */
static void ClearIRCounters() {
    int i;
    for(i = 0; i < SENSORCOUNT; i++) {
#ifndef IR_ADC
        irCounter[i] = 0;
#else
        printf("RESET!\n");
        irReading[i] = 0;
#endif
    }

}

/**
 * @Function: SwitchIR
 * @param select, MAIN, ANGLE, or OFF
 * @remark Switch between the main and angle beacons, and activates
 *         the appropriate select port.
 * @date 2012.3.5 12:47 */
static void SwitchIR(unsigned int select) {
    /*
    if (select == MAIN) {
        mainIsSelected = 1
    }
    else
        mainIsSelected = 0;
     */

    
    if (select == ANGLE) {
        IR_MAIN_SELECT = !0;
        IR_ANGLE_SELECT = !1;
    }
    else if (select == MAIN) {
        IR_ANGLE_SELECT = !0;
        IR_MAIN_SELECT = !1;
    }
    else {
        IR_MAIN_SELECT = !0;
        IR_ANGLE_SELECT = !0;
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
    #ifndef IR_ADC
    IR_READ_TRIS = 1;
    #endif
    IR_MAIN_SELECT_TRIS = 0;
    IR_ANGLE_SELECT_TRIS = 0;

    // Enable the main beacon
    SwitchIR(OFF);
    mainIsSelected = 1;
    irState = waitOff;


    dbprintf("\nIR sensors initialized (%d)", SENSORCOUNT);

    return SUCCESS;

}

char IR_HandleSM() {
    switch (irState) {
        case read:
            // read a sensor
            UpdateIRCounter();
            
            timesRead++;
            if (timesRead >= COUNTMAX) {
                UpdateIRState();
                // switch beacons
                ClearIRCounters();
                timesRead = 0;
                SwitchIR(OFF);
                mainIsSelected ^= 1;
                irState = waitOff;
                InitTimer(TIMER_NUM, UPDATE_DELAY);
                break;
            }
    
            InitTimer(TIMER_NUM, UPDATE_DELAY);
            irState = waitOn;
            break;

        case waitOff:
            if (IsTimerExpired(TIMER_NUM)) {
                //printf("\nExpired!!");
                if (mainIsSelected) {
                    SwitchIR(MAIN);
                }
                else {
                    SwitchIR(ANGLE);
                }
                InitTimer(TIMER_NUM, UPDATE_DELAY);
                irState = waitOn;
            }

            break;
        case waitOn:
            if (IsTimerExpired(TIMER_NUM)) {
                //printf("\nExpired!!");
                irState = read;
            }

            break;

        default:
            dbprintf("\nHorrible Error");
            return ERROR;
            break;
    }
    return SUCCESS;
}

char IR_MainTriggered() {
    return irFound[IR_MAIN_I];
}

char IR_AngleTriggered() {
    return irFound[IR_ANGLE_I];
}

char IR_MainReading() {
    return irPort[IR_MAIN_I];
}

char IR_AngleReading() {
    return irPort[IR_ANGLE_I];
}

char IR_End() {
    StopTimer(TIMER_NUM);
    SwitchIR(OFF);

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
    TIMERS_Init();
    SERIAL_Init();
    int i = 0;
    char mainSelected;
    char angleSelected;

    INTEnableSystemMultiVectoredInt();
    IR_Init();
    AD_Init( IR_READ );

    while(1) {
        IR_HandleSM();
/*
        if (mainIsSelected) {
            mainSelected = 'x';
            angleSelected = '_';
        }
        else {
            mainSelected = '_';
            angleSelected = 'x';
        }
 * */

         //printf("\nTrig: %cMain=%x, %cAngle=%x", mainSelected, IR_MainTriggered(),
         //   angleSelected, IR_AngleTriggered());
#ifndef IR_ADC
         printf("\nCount: Main=%x, Angle=%x", irCounter[IR_MAIN_I], irCounter[IR_ANGLE_I]);
#else
        // printf("\nADC: %cMain=%x, %cAngle=%x",mainSelected, irPort[IR_MAIN_I],
         //angleSelected, irPort[IR_ANGLE_I]);
         if (i >= 100) {
            //printf("\nADC: %x", ReadADPin(IR_READ));
            i = 0;
         }
         i++;
#endif
         //printf("\nReading: Main=%x, Angle=%x\n", irPort[IR_MAIN_I], irPort[IR_ANGLE_I]);


        while (!IsTransmitEmpty()); // bad, this is blocking code
    } // end of loop


    IR_End();
}
#endif

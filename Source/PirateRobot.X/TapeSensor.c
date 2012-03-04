/*
 * File:   TapeSensor.c
 * Author: dagoodma
 *
 * Taking 3 samples, at 
 *
 */

#include <p32xxxx.h>
#include <plib.h>
#include "serial.h"
#include "timers.h"
#include "PORTS.h"
#include "pwm.h"
#include "AD.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

//#define USE_LEDS

#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define TAPESENSORCOUNT 7
#define TAPEPINCOUNT (TAPESENSORCOUNT * 2)
#define FULLOVERFLOW 0xFFFF
#define TAPEPERIODTIME 2000 // usec (500 Hz)
#define TIMESTOREAD 3

// Delay times
#define READDELAY 500 // usec
#define STARTDELAY (TAPEPERIODTIME - (READDELAY * TIMESTOREAD)) // usec

// Sensor thresholds
#define ONTAPE_THRESHOLD 0x92
#define OFFTAPE_THRESHOLD 0xAF

/* Note that you need to set the prescalar and periferal clock appropriate to
 * the processor board that you are using. In order to calculate the minimal
 * prescalar: Prescalar = (2000*F_PB/(1000000*0xFFFF))+1, round down */
#ifndef F_CPU
#define F_CPU       80000000L
#define F_PB        (F_CPU/2)
#define F_PB_IN_KHZ (F_PB/1000)
#define PRESCALE    2
#define uSEC        (F_PB_IN_KHZ / (PRESCALE * 1000))
#endif


/* Tape sensor port configuration 
    TAPE_LEFT 0x001    ===> PortV-5
    TAPE_CENTER 0x002  ===> PortV-6
    TAPE_RIGHT 0x004   ===> PortV-7
    TAPE_BACK 0x008    ===> PortV-8
    TAPE_ARMFRONT 0x010===> PortW-3
    TAPE_ARMLEFT 0x020 ===> PortW-4
    TAPE_ARMRIGHT 0x040===> PortW-5

    LED_
*/



//--------------- Photodetectors --------------
#define TAPE_LEFT   AD_PORTV5
#define TAPE_CENTER AD_PORTV6
#define TAPE_RIGHT AD_PORTV7
#define TAPE_BACK AD_PORTV8
#define TAPE_ARMFRONT AD_PORTW3
#define TAPE_ARMLEFT AD_PORTW4
#define TAPE_ARMRIGHT AD_PORTW5

//----------------- Emitters ------------------
#define TAPE_LED_TRIS PORTZ07_TRIS
#define TAPE_LED_LAT PORTZ07_LAT
#define TAPE_LEDS1_TRIS PORTZ10_TRIS
#define TAPE_LEDS1_LAT PORTZ10_LAT
#define TAPE_LEDS2_TRIS PORTZ11_TRIS
#define TAPE_LEDS2_LAT PORTZ11_LAT
#define TAPE_LEDS3_TRIS PORTZ12_TRIS
#define TAPE_LEDS3_LAT PORTZ12_LAT

/*
//----------------- Uno32 LEDs ----------------
#define LED_BANK1_3 LATDbits.LATD6
#define LED_BANK1_2 LATDbits.LATD11
#define LED_BANK1_1 LATDbits.LATD3
#define LED_BANK1_0 LATDbits.LATD5

#define LED_BANK2_3 LATFbits.LATF6
#define LED_BANK2_2 LATGbits.LATG7
#define LED_BANK2_1 LATDbits.LATD7
#define LED_BANK2_0 LATGbits.LATG8

#define LED_BANK3_3 LATBbits.LATB0
#define LED_BANK3_2 LATFbits.LATF5
#define LED_BANK3_1 LATFbits.LATF4
#define LED_BANK3_0 LATGbits.LATG6
*/


/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/
static int uSecondsLeftToGo = 0;
static unsigned short int lastTime = 0;
static unsigned short int timesRead = 0;
static char ledsOn = 0;

enum tapeSensorIndex { TAPE_LEFT_I, TAPE_CENTER_I, TAPE_RIGHT_I, TAPE_BACK_I, 
    TAPE_ARMFRONT_I, TAPE_ARMLEFT_I, TAPE_ARMRIGHT_I };

static unsigned int const tapeSensorPortMap[] = {TAPE_LEFT, TAPE_CENTER,
    TAPE_RIGHT, TAPE_BACK, TAPE_ARMFRONT, TAPE_ARMLEFT, TAPE_ARMRIGHT };



static char tapeSensorStates[TAPESENSORCOUNT];
static unsigned int tapeSensorOffReadings[TAPESENSORCOUNT][TIMESTOREAD];
static unsigned int tapeSensorOnReadings[TAPESENSORCOUNT][TIMESTOREAD];

static enum {off, init, read, on} tapeState;


/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/
static char ReadTapeSensors();
static void ledsAllOn();
static void ledsAllOff();
static char IsOnTape(unsigned int value);


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * Function: ReadTapeSensors
 * @return SUCCESS or ERROR
 * @remark Takes a reading from the tape sensors and records it. When
 *         the third reading is taken the state of each sensor will be
 *         updated.
 */
static char ReadTapeSensors() {
    if (timesRead > TIMESTOREAD) {
        dbprintf("\nRead Error, too many times");
        return ERROR;
    }
    unsigned short int j  = timesRead;

    // Read each of the sensors
    unsigned short int i;
    for (i = 0; i <= TAPESENSORCOUNT; i++) {
        unsigned int reading = ReadADPin(tapeSensorPortMap[i]);
        if (ledsOn) {
            tapeSensorOnReadings[i][j] = reading;
        }
        else {
            tapeSensorOffReadings[i][j] = reading;
        }
    }

    // Update our tape sensor states if we have enough readings
    if (timesRead == TIMESTOREAD) {
        unsigned short int i;
        for (i = 0; i <= TAPESENSORCOUNT; i++) {
            char timesTapeSeen = 0;
            char tapeSeenResult = FALSE;
            unsigned short int j;
            for (j = 0; j <= TIMESTOREAD; i++) {
                unsigned int normalReading
                    = tapeSensorOnReadings[i][j] - tapeSensorOffReadings[i][j];
                if (IsOnTape(normalReading))
                    timesTapeSeen += 1;
            }
        
            // Use majority of readings 
            if (timesTapeSeen >= ((TIMESTOREAD / 2) + 1)) 
                tapeSeenResult = 1;

            tapeSensorStates[i] = tapeSeenResult;

            #ifdef USE_LEDS
            ledPortMap[i] = tapeSensorStates[i];
            #endif
        }
    } // if TIMESTOREAD

    return SUCCESS;
} // ReadTapeSensors()


/**
 * Function: ledsAllOn
 * @remark Turns on all of the tape sensor's emitter LEDs.
 */
static void ledsAllOn() {
    TAPE_LED_LAT = 1;
    TAPE_LEDS1_LAT = 1;
    TAPE_LEDS2_LAT = 1;
    TAPE_LEDS3_LAT = 1;
}


/**
 * Function: ledsAllOff
 * @remark Turns off all of the tape sensor's emitter LEDs.
 */
static void ledsAllOff() {
    TAPE_LED_LAT = 0;
    TAPE_LEDS1_LAT = 0;
    TAPE_LEDS2_LAT = 0;
    TAPE_LEDS3_LAT = 0;
}

/**
 * Function: 
 * @remark Turns off all of the tape sensor's emitter LEDs.
 */
static char IsOnTape(unsigned int value) {
    static unsigned int threshold = ONTAPE_THRESHOLD;

    if (value <= threshold ) {
        return TRUE;
        threshold = OFFTAPE_THRESHOLD;
    }

    return FALSE;
}



/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

char Tape_Init(unsigned short int tapePins) {
    unsigned short int CurrentTime;
    tapeState = init;
    dbprintf("\nInitializing the Tape Sensor Module.");

    // Define inputs
    AD_Init(TAPE_LEFT | TAPE_CENTER | TAPE_RIGHT | TAPE_BACK |
        TAPE_ARMFRONT | TAPE_ARMLEFT | TAPE_ARMRIGHT);

    // Define outputs (LEDs)
    TAPE_LED_TRIS = 0;
    TAPE_LEDS1_TRIS = 0;
    TAPE_LEDS2_TRIS = 0;
    TAPE_LEDS3_TRIS = 0;

    OpenTimer5(T5_ON | T5_IDLE_STOP | T5_GATE_OFF | T5_PS_1_2 | T5_SOURCE_INT, TAPEPERIODTIME * uSEC);
    ConfigIntTimer5(T5_INT_OFF);

    // State variables
    ledsOn = 0;
    timesRead = 0;
    uSecondsLeftToGo = STARTDELAY;
    dbprintf("\nTapesensors initialized (%d)", TAPESENSORCOUNT);


    tapeState = off;


    return SUCCESS;
}

char Tape_HandleSM() {
    unsigned short int currentTime = ReadTimer5();
    uSecondsLeftToGo -= (currentTime - lastTime);
    lastTime = currentTime;

    switch (tapeState) {
        case off:
            if (uSecondsLeftToGo <= 0) {
                if (timesRead < TIMESTOREAD) {
                    tapeState = read;
                }
                else {
                    ledsOn = 1;
                    uSecondsLeftToGo = STARTDELAY;
                    timesRead = 0;
                    lastTime = 0; // clear because of overflow
                    tapeState = on;
                    ledsAllOn();
                }
            }
            break;
        case read:
            ReadTapeSensors();
            timesRead++;

            if (!ledsOn) {
                // Only delay if we will read another time
                if (timesRead < 3) 
                    uSecondsLeftToGo = READDELAY;
                tapeState = off;
            }
            else {
                // Only delay if we will read another time
                if (timesRead < 3) 
                    uSecondsLeftToGo = READDELAY;
                tapeState = on;
            }
            break;
        case on:
            if (uSecondsLeftToGo <= 0) {
                if (timesRead < TIMESTOREAD) {
                    tapeState = read;
                }
                else {
                    ledsOn = 0;
                    ledsAllOff();
                    uSecondsLeftToGo = STARTDELAY;
                    timesRead = 0;
                    lastTime = 0; // clear because of overflow
                    tapeState = off;
                }
            }
            break;
        default:
            dbprintf("\nHorrible Error");
            break;
    } // switch

} // Tape_HandleSM

char Tape_End() {
    AD_End();

    return SUCCESS;
}


// ********************* Tape Sensor Accessors *************************
char Tape_LeftTriggered() { return tapeSensorStates[TAPE_LEFT_I]; }

char Tape_CenterTriggered() { return tapeSensorStates[TAPE_CENTER_I]; }

char Tape_RightTriggered() { return tapeSensorStates[TAPE_RIGHT_I]; }

char Tape_BackTriggered() { return tapeSensorStates[TAPE_BACK_I]; }

char Tape_ArmFrontTriggered() { return tapeSensorStates[TAPE_ARMFRONT_I]; }

char Tape_ArmLeftTriggered() { return tapeSensorStates[TAPE_ARMLEFT_I]; }

char Tape_ArmRightTriggered() { return tapeSensorStates[TAPE_ARMRIGHT_I]; }

/*******************************************************************************
 * TEST HARNESS                                                                *
 ******************************************************************************/
#ifdef TAPE_TEST


#define DELAY() for(i=0;i < NOPCOUNT; i++) __asm("nop")
#define NOPCOUNT 990000

int main(void) {
    SERIAL_Init();
    int i = 0;

    INTEnableSystemMultiVectoredInt();

    if (Tape_Init() == SUCCESS) {
        printf("\nSuccesfully initialized tape sensors (%d Hz)",TAPEPERIODTIME);
    }
    else {
        printf("\nFailed to initialize tape sensors");
        return 1;
    }

    // Test routine (above goes in init)
    printf("\nHello tester!");
    DELAY();

    while(1) {
        Tape_HandleSM();

        if (i >= 100000) {
            printf("\nStates: LEFT=(%x), CENTER=(%x), RIGHT=(%x), BACK=(%x), \
               ARM_FRONT=(%x), ARM_LEFT=(%x), ARM_RIGHT=(%x)",
               Tape_LeftTriggered(), Tape_CenterTriggered(),
               Tape_RightTriggered(), Tape_BackTriggered(),
               Tape_ArmFrontTriggered(), TapeArmLeftTriggered(),
               Tape_ArmRightTriggered());
               
            i = 0;
            while (!IsTransmitEmpty()); // bad, this is blocking code
        }
        i++;
    } // end of loop

    Tape_End();
    return 0;
} // test harness
#endif


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
#include "PORTS.h"
#include "AD.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define TAPE_TEST
//#define USE_LEDS

//#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define TAPESENSORCOUNT 7
#define TAPEPINCOUNT (TAPESENSORCOUNT * 2)

#define TIMESTOREAD 4

// Timer config
#define TIMER_NUM 2
#define READDELAY 1 // msec
#define STARTDELAY 1 // msec

// Sensor thresholds
#define ONTAPE_THRESHOLD 0x31
#define OFFTAPE_THRESHOLD 0x42

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
#define TAPE_LEDS1_TRIS PORTY08_TRIS
#define TAPE_LEDS1_LAT PORTY08_LAT
#define TAPE_LEDS2_TRIS PORTW06_TRIS
#define TAPE_LEDS2_LAT PORTW06_LAT
#define TAPE_LEDS3_TRIS PORTZ11_TRIS
#define TAPE_LEDS3_LAT PORTZ11_LAT

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
//static int uSecondsLeftToGo = 0;
static unsigned short int lastTime = 0;
static unsigned short int timesRead = 0;
static char ledsOn = 0;

enum sensorIndex { TAPE_LEFT_I, TAPE_CENTER_I, TAPE_RIGHT_I, TAPE_BACK_I,
    TAPE_ARMFRONT_I, TAPE_ARMLEFT_I, TAPE_ARMRIGHT_I };

static unsigned int const sensorPortMap[] = {TAPE_LEFT, TAPE_CENTER,
    TAPE_RIGHT, TAPE_BACK, TAPE_ARMFRONT, TAPE_ARMLEFT, TAPE_ARMRIGHT };



//static char sensorStates[TAPESENSORCOUNT];
static unsigned int sensorOffReadings[TAPESENSORCOUNT];
static unsigned int sensorOnReadings[TAPESENSORCOUNT];
static unsigned int sensorReading[TAPESENSORCOUNT];

static enum {off, init, read, on} tapeState;


/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/
static char ReadTapeSensors();
static void ledsAllOn();
static void ledsAllOff();
static char IsOnTape(unsigned int value);
static void ClearReadings();
static void UpdateReadings();
static unsigned int LeftReading();
static unsigned int CenterReading();
static unsigned int RightReading();
static unsigned int BackReading();
static unsigned int ArmFrontReading();
static unsigned int ArmLeftReading();
static unsigned int ArmRightReading();


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

    // Read each of the sensors
    unsigned short int i;
    for (i = 0; i < TAPESENSORCOUNT; i++) {
        unsigned int reading = ReadADPin(sensorPortMap[i]);
        if (ledsOn) {
            sensorOnReadings[i] += reading;
        }
        else {
            sensorOffReadings[i] += reading;
        }
    }

    return SUCCESS;
} // ReadTapeSensors()


/**
 * Function: ledsAllOn
 * @remark Turns on all of the tape sensor's emitter LEDs.
 */
static void ledsAllOn() {
    ledsOn = 1;
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
    ledsOn = 0;
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
    //static unsigned int threshold = ONTAPE_THRESHOLD;
    static unsigned int threshold = ONTAPE_THRESHOLD;

    if (value <= threshold ) {
        //threshold = OFFTAPE_THRESHOLD;
        return TRUE;
    }

    //threshold = ONTAPE_THRESHOLD;
    return FALSE;
}

static void ClearReadings() {
    int i;
    for (i = 0; i < TAPESENSORCOUNT; i++) {
        sensorOffReadings[i] = 0;
        sensorOnReadings[i] = 0;
    }
}

static void UpdateReadings() {
    int i;
    for (i = 0; i < TAPESENSORCOUNT; i++) {
        if (!ledsOn) {
            sensorOffReadings[i] >>= 2;
        }
        else {
            sensorOnReadings[i] >>= 2;
            sensorReading[i] = sensorOnReadings[i] - sensorOffReadings[i];
            sensorOnReadings[i] = 0;
            sensorOffReadings[i] = 0;
        }
    }
    
}



/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

char Tape_Init() {
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

    InitTimer(TIMER_NUM, STARTDELAY);

    ClearReadings();

    // State variables
    ledsOn = 0;
    timesRead = 0;
    dbprintf("\nTapesensors initialized (%d)", TAPESENSORCOUNT);


    tapeState = off;


    return SUCCESS;
}

char Tape_HandleSM() {
    dbprintf("\nState=%d, leds=%x", tapeState,ledsOn);
    switch (tapeState) {
        
        case off:
            dbprintf("\noff!");
            if (IsTimerExpired(TIMER_NUM)) {
                if (timesRead < TIMESTOREAD) {
                    tapeState = read;
                }
                else {
                    InitTimer(TIMER_NUM, STARTDELAY);
                    timesRead = 0;
                    tapeState = on;
                    UpdateReadings();
                    ledsAllOn();
                }
            }
            break;
        case read:
           dbprintf("\nread!");
           ReadTapeSensors();
            
            // Only delay if we will read another time
            if (timesRead < TIMESTOREAD)
                InitTimer(TIMER_NUM, READDELAY);

            if (!ledsOn) {
                tapeState = off;
            }
            else {
                tapeState = on;
            }
            timesRead++;
            break;
        case on:
          dbprintf("\non!");
            if (IsTimerExpired(TIMER_NUM)) {
                if (timesRead < TIMESTOREAD) {
                    tapeState = read;
                }
                else {
                    InitTimer(TIMER_NUM, STARTDELAY);
                    timesRead = 0;
                    tapeState = off;
                    UpdateReadings();
                    ledsAllOff();
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
char Tape_LeftTriggered() { return IsOnTape(LeftReading()); }

char Tape_CenterTriggered() { return IsOnTape(CenterReading()); }

char Tape_RightTriggered() { return IsOnTape(RightReading()); }

char Tape_BackTriggered() { return IsOnTape(BackReading()); }

char Tape_ArmFrontTriggered() { return IsOnTape(ArmFrontReading()); }

char Tape_ArmLeftTriggered() { return IsOnTape(ArmLeftReading()); }

char Tape_ArmRightTriggered() { return IsOnTape(ArmRightReading()); }


static unsigned int LeftReading() { return sensorReading[TAPE_LEFT_I]; }

static unsigned int CenterReading() { return sensorReading[TAPE_CENTER_I]; }

static unsigned int RightReading() { return sensorReading[TAPE_RIGHT_I]; }

static unsigned int BackReading() { return sensorReading[TAPE_BACK_I]; }

static unsigned int ArmFrontReading() { return sensorReading[TAPE_ARMFRONT_I]; }

static unsigned int ArmLeftReading() { return sensorReading[TAPE_ARMLEFT_I]; }

static unsigned int ArmRightReading() { return sensorReading[TAPE_ARMRIGHT_I]; }


/*******************************************************************************
 * TEST HARNESS                                                                *
 ******************************************************************************/
#ifdef TAPE_TEST


#define DELAY() for(i=0;i < NOPCOUNT; i++) __asm("nop")
#define NOPCOUNT 990000

int main(void) {
    TIMERS_Init();
    SERIAL_Init();
    int i = 0;
    unsigned int index = TAPE_LEFT_I;

    INTEnableSystemMultiVectoredInt();

    if (Tape_Init() == SUCCESS) {
        printf("\nSuccesfully initialized tape sensors");
    }
    else {
        printf("\nFailed to initialize tape sensors");
        return 1;
    }

    printf("\nHello tester! Use i to change tape sensors.")
    DELAY();

    while(1) {
        Tape_HandleSM();
        

        printf("\nSensor #%u: ADC=%x, LED_OFF=%x, LED_ON=%x, AVG=%x", index,
            ReadADPin(sensorPortMap[index]), sensorOffReadings[index],
            sensorOnReadings[index], sensorReading[index]);
            
         
        if (keyPressed != 0) {
            if (keyPressed == 'i') {
                print("\nSelect a sensor:");
                printf("\n0=LEFT, 1=CENTER, 2=RIGHT, 3=BACK, 4=ARMFRONT, 5=ARMLEFT, 6=ARMRIGHT");
                char try = GetChar();
                if (try <= 54 && try >= 48)
                    index = atoi(try);
                else 
                    printf("\nInvalid sensor selected");
             }
        }

        while (!IsTransmitEmpty()); // bad, this is blocking code
    } // end of loop

    Tape_End();
    return 0;
} // test harness
#endif


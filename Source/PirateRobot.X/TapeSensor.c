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
#include "LED.h"
#include "TapeSensor.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
//#define TAPE_TEST
//#define TAPE_PRINT
//#define TAPE_TEST_ALL
#define USE_LEDS

//#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define TAPESENSORCOUNT 7
#define TAPEPINCOUNT (TAPESENSORCOUNT * 2)

#define TIMESTOREAD 2 // MUST BE EVEN #
#define TIMESTOSHIFT 1 // divides by 2^2 == 4

#define TIMESTOREAD2 3

// Timer config
#define TIMER_NUM 3
#define READDELAY 7 // msec
#define STARTDELAY 3 // msec

// Sensor thresholds
#ifdef TAPE_TEST
#define ONTAPE_THRESHOLD 0x22
#define OFFTAPE_THRESHOLD 0x30
#else
#define ONTAPE_THRESHOLD 0x22
#define OFFTAPE_THRESHOLD 0x35
#endif

//--------------- Photodetectors --------------
//************* MOVED TO TapeSensor.h ***************
/* 
#define TAPE_LEFT   AD_PORTV5
#define TAPE_CENTER AD_PORTV6
#define TAPE_RIGHT AD_PORTV7
#define TAPE_BACK AD_PORTV8
#define TAPE_ARMFRONT AD_PORTW3
#define TAPE_ARMLEFT AD_PORTW4
#define TAPE_ARMRIGHT AD_PORTW5
*/

//----------------- Emitters ------------------
#define TAPE_LED_TRIS PORTZ07_TRIS
#define TAPE_LED_LAT PORTZ07_LAT
#define TAPE_LEDS1_TRIS PORTY08_TRIS
#define TAPE_LEDS1_LAT PORTY08_LAT
#define TAPE_LEDS2_TRIS PORTX08_TRIS
#define TAPE_LEDS2_LAT PORTX08_LAT
#define TAPE_LEDS3_TRIS PORTZ11_TRIS
#define TAPE_LEDS3_LAT PORTZ11_LAT



/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/
//static int uSecondsLeftToGo = 0;
static unsigned short int lastTime = 0;
static unsigned short int timesRead = 0;
static unsigned short int timesRead2 = 0;
static char ledsOn = 0;



static unsigned int const sensorPortMap[] = { TAPE_LEFT, TAPE_CENTER,
    TAPE_RIGHT, TAPE_BACK, TAPE_ARMFRONT, TAPE_ARMLEFT, TAPE_ARMRIGHT };



//static char sensorStates[TAPESENSORCOUNT];
static unsigned int sensorOffReadings[TAPESENSORCOUNT];
static unsigned int sensorOnReadings[TAPESENSORCOUNT];
static unsigned int sensorHighs[] = { 0, 0, 0, 0, 0, 0, 0 };
static unsigned int sensorState[TAPESENSORCOUNT];
static unsigned int sensorThreshold[] = {ONTAPE_THRESHOLD, ONTAPE_THRESHOLD,
    ONTAPE_THRESHOLD, ONTAPE_THRESHOLD, ONTAPE_THRESHOLD, ONTAPE_THRESHOLD,
    ONTAPE_THRESHOLD };
static unsigned int sensorReading[TAPESENSORCOUNT];

static unsigned int onTapeThreshold = ONTAPE_THRESHOLD;
static unsigned int offTapeThreshold = OFFTAPE_THRESHOLD;

static enum {off, init, read, on} tapeState;


/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/
static char ReadTapeSensors();
static void ledsAllOn();
static void ledsAllOff();
static char IsOnTape(unsigned int value, unsigned int threshold);
static void ClearReadings();
static void UpdateReadings();
void DebugLEDOn(unsigned int index);
void DebugLEDOff(unsigned int index);
static unsigned int LeftReading();
static unsigned int CenterReading();
static unsigned int RightReading();
static unsigned int BackReading();
static unsigned int ArmFrontReading();
static unsigned int ArmLeftReading();
static unsigned int ArmRightReading();
static void UpdateThresholds(unsigned int newOnTapeThreshold, unsigned int newOffTapeThreshold);


/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * Function: ReadTapeSensors
 * @return SUCCESS or ERROR
 * @remark Reads the AD value from each tape sensor and adds it to the
 *         corresponding reading variable.
 * @date 2012.3.5 12:13 */
static char ReadTapeSensors() {
    if (timesRead > TIMESTOREAD) {
        //dbprintf("\nRead Error, too many times");
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
 * @date 2012.3.5 12:13 */
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
 * @date 2012.3.5 12:13 */
static void ledsAllOff() {
    ledsOn = 0;
    TAPE_LED_LAT = 0;
    TAPE_LEDS1_LAT = 0;
    TAPE_LEDS2_LAT = 0;
    TAPE_LEDS3_LAT = 0;
}

/**
 * Function: IsOnTape
 * @remark Compares the value with the given threshold. TRUE when
 *         less than or equal to threshold.
 * @date 2012.3.5 12:13 */
static char IsOnTape(unsigned int value, unsigned int threshold) {

    if (value <= threshold) {
        return TRUE;
    }
    else 
    return FALSE;
}

/**
 * Function: ClearReadings
 * @remark Iterates through the tape sensor readings and clears them.
 *         Note: This is only used by Tape_Init since UpdateReadings()
 *               already does this.
 * @date 2012.3.5 12:13 */
static void ClearReadings() {
    int i;
    for (i = 0; i < TAPESENSORCOUNT; i++) {
        sensorOffReadings[i] = 0;
        sensorOnReadings[i] = 0;
    }
}

/**
 * Function: UpdateReadings
 * @remark Iterates through the tape sensors and divides each reading
 *   by TIMESTOREAD, and when the leds are on the difference is taken
 *   and the temporary readings are cleared.
 * @date 2012.3.5 12:13 */
static void UpdateReadings() {
    int i;
    if (ledsOn)
        timesRead2++;
    for (i = 0; i < TAPESENSORCOUNT; i++) {
        if (!ledsOn) {
            sensorOffReadings[i] >>= TIMESTOSHIFT;
        }
        else {
            sensorOnReadings[i] >>= TIMESTOSHIFT;
            sensorReading[i] = sensorOnReadings[i] - sensorOffReadings[i];

            char result = IsOnTape(sensorReading[i], sensorThreshold[i]);
            // hysteresis
            if (result)
                sensorThreshold[i] = offTapeThreshold;
            else
                sensorThreshold[i] = onTapeThreshold;

            sensorHighs[i] += result;

            if (timesRead2 >= TIMESTOREAD2) {
#ifdef TAPE_PRINT
                printf("\n%i: Avg=%d, onAvg=%d, offAvg=%d, highs=%d",i, sensorReading[i], sensorOnReadings[i], sensorOffReadings[i],sensorHighs[i]);
#endif

                char state = sensorHighs[i] >= TIMESTOREAD2;

                #ifdef USE_LEDS
                if (state)
                    DebugLEDOn(i);
                else
                    DebugLEDOff(i);
                #endif

                sensorState[i] = state;
                sensorHighs[i] = 0;
            }
            
            sensorOnReadings[i] = 0;
            sensorOffReadings[i] = 0;
        }
    } // for
    if (timesRead2 >= TIMESTOREAD2)
        timesRead2 = 0;
}

void DebugLEDOn(unsigned int index) {

    switch (index) {
        case TAPE_LEFT_I:
            LED_OnBank(LED_BANK3, 0x8);
            break;
        case TAPE_CENTER_I:
            LED_OnBank(LED_BANK3, 0x4);
            break;
        case TAPE_RIGHT_I:
            LED_OnBank(LED_BANK3, 0x2);
            break;
        case TAPE_BACK_I:
            LED_OnBank(LED_BANK1, 0x4);
            break;
        case TAPE_ARMFRONT_I:
            LED_OnBank(LED_BANK3, 0x1);
            break;
        case TAPE_ARMLEFT_I:
            LED_OnBank(LED_BANK1, 0x2);
            break;
        case TAPE_ARMRIGHT_I:
            LED_OnBank(LED_BANK1, 0x1);
            break;
    } // switch
}

void DebugLEDOff(unsigned int index) {

    switch (index) {
        case TAPE_LEFT_I:
            LED_OffBank(LED_BANK3, 0x8);
            break;
        case TAPE_CENTER_I:
            LED_OffBank(LED_BANK3, 0x4);
            break;
        case TAPE_RIGHT_I:
            LED_OffBank(LED_BANK3, 0x2);
            break;
        case TAPE_BACK_I:
            LED_OffBank(LED_BANK1, 0x4);
            break;
        case TAPE_ARMFRONT_I:
            LED_OffBank(LED_BANK3, 0x1);
            break;
        case TAPE_ARMLEFT_I:
            LED_OffBank(LED_BANK1, 0x2);
            break;
        case TAPE_ARMRIGHT_I:
            LED_OffBank(LED_BANK1, 0x1);
            break;
    } // switch
}


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

char Tape_Init() {
    tapeState = init;
    //dbprintf("\nInitializing the Tape Sensor Module.");

    // Define inputs

    // Define outputs (LEDs)
    TAPE_LED_TRIS = 0;
    TAPE_LEDS1_TRIS = 0;
    TAPE_LEDS2_TRIS = 0;
    TAPE_LEDS3_TRIS = 0;

    InitTimer(TIMER_NUM, STARTDELAY);
#ifdef USE_LEDS
    LED_Init(LED_BANK3 |LED_BANK1);
    LED_OffBank(LED_BANK3, 0xF);
    LED_OffBank(LED_BANK1, 0xF);

#endif

    ClearReadings();

    // State variables
    ledsOn = 0;
    timesRead = 0;
    //dbprintf("\nTapesensors initialized (%d)", TAPESENSORCOUNT);


    tapeState = off;


    return SUCCESS;
}

char Tape_HandleSM() {
    //dbprintf("\nState=%d, leds=%x", tapeState,ledsOn);
    LED_OnBank(LED_BANK1, 0x8);
    switch (tapeState) {
        
        case off:
            //dbprintf("\noff!");
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
           //dbprintf("\nread!");
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
          //dbprintf("\non!");
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
            //dbprintf("\nHorrible Error");
            break;
    } // switch
    LED_OffBank(LED_BANK1, 0x8);
#ifdef TAPE_PRINT

    /*
    printf("\n left TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_LeftTriggered(),
            ReadADPin(sensorPortMap[TAPE_LEFT_I]),
            sensorOffReadings[TAPE_LEFT_I],
            sensorOnReadings[TAPE_LEFT_I], sensorReading[TAPE_LEFT_I]);
        printf("\n center TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_CenterTriggered(),
            ReadADPin(sensorPortMap[TAPE_CENTER_I]),
            sensorOffReadings[TAPE_CENTER_I],
            sensorOnReadings[TAPE_CENTER_I], sensorReading[TAPE_CENTER_I]);
        printf("\n right TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_RightTriggered(),
            ReadADPin(sensorPortMap[TAPE_RIGHT_I]),
            sensorOffReadings[TAPE_RIGHT_I],
            sensorOnReadings[TAPE_RIGHT_I], sensorReading[TAPE_RIGHT_I]);
        printf("\n back TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_BackTriggered(),
            ReadADPin(sensorPortMap[TAPE_BACK_I]),
            sensorOffReadings[TAPE_BACK_I],
            sensorOnReadings[TAPE_BACK_I], sensorReading[TAPE_BACK_I]);
        printf("\n armleft TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_ArmLeftTriggered(),
            ReadADPin(sensorPortMap[TAPE_ARMLEFT_I]),
            sensorOffReadings[TAPE_ARMLEFT_I],
            sensorOnReadings[TAPE_ARMLEFT_I], sensorReading[TAPE_ARMLEFT_I]);
        printf("\n armright TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_ArmRightTriggered(),
            ReadADPin(sensorPortMap[TAPE_ARMRIGHT_I]),
            sensorOffReadings[TAPE_ARMRIGHT_I],
            sensorOnReadings[TAPE_ARMRIGHT_I], sensorReading[TAPE_ARMRIGHT_I]);
        printf("\n armfront TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_ArmFrontTriggered(),
            ReadADPin(sensorPortMap[TAPE_ARMFRONT_I]),
            sensorOffReadings[TAPE_ARMFRONT_I],
            sensorOnReadings[TAPE_ARMFRONT_I], sensorReading[TAPE_ARMFRONT_I]);
     */
#endif

} // Tape_HandleSM

char Tape_End() {
    AD_End();

    return SUCCESS;
}

void Tape_SetOnTapeThreshold(unsigned int index) {
    unsigned int newOnTapeThreshold = sensorReading[index]+5;
    printf("\nNew on tape threshold=%d",newOnTapeThreshold);
    UpdateThresholds(newOnTapeThreshold, offTapeThreshold);
}
void Tape_SetOffTapeThreshold(unsigned int index) {
    //unsigned int newOffTapeThreshold = onTapeThreshold
    //    + ((sensorReading[index] - onTapeThreshold) / 3);
    unsigned int newOffTapeThreshold = sensorReading[index]-5;


    printf("\nNew off tape threshold=%d",newOffTapeThreshold);
    UpdateThresholds(onTapeThreshold, newOffTapeThreshold);
}

static void UpdateThresholds(unsigned int newOnTapeThreshold, unsigned int newOffTapeThreshold) {
    int i;
    for (i = 0; i < TAPESENSORCOUNT; i++) {
        if (sensorThreshold[i] == onTapeThreshold)
            sensorThreshold[i] = newOnTapeThreshold;
        else
            sensorThreshold[i] = newOffTapeThreshold;
    }
    onTapeThreshold = newOnTapeThreshold;
    offTapeThreshold = newOffTapeThreshold;
}


// ********************* Tape Sensor Accessors *************************
char Tape_LeftTriggered() { return sensorState[TAPE_LEFT_I]; }

char Tape_CenterTriggered() { return sensorState[TAPE_CENTER_I]; }

char Tape_RightTriggered() { return sensorState[TAPE_RIGHT_I]; }

char Tape_BackTriggered() { return sensorState[TAPE_BACK_I]; }

char Tape_ArmFrontTriggered() { return sensorState[TAPE_ARMFRONT_I]; }

char Tape_ArmLeftTriggered() { return sensorState[TAPE_ARMLEFT_I]; }

char Tape_ArmRightTriggered() { return sensorState[TAPE_ARMRIGHT_I]; }

char Tape_AnyTriggered() {
    return Tape_ArmRightTriggered() || Tape_ArmLeftTriggered() ||
        Tape_ArmFrontTriggered() || Tape_LeftTriggered() ||
        Tape_CenterTriggered() || Tape_RightTriggered() ||
        Tape_BackTriggered();
}

char Tape_AnyRightTriggered() {
    return Tape_RightTriggered() || Tape_ArmLeftTriggered() ||
            Tape_ArmFrontTriggered() || Tape_ArmRightTriggered();
}

char Tape_AnyFrontTriggered() {
    return Tape_AnyRightTriggered || Tape_CenterTriggered || Tape_LeftTriggered;
}

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
    AD_Init(TAPE_LEFT | TAPE_CENTER | TAPE_RIGHT | TAPE_BACK |
        TAPE_ARMFRONT | TAPE_ARMLEFT | TAPE_ARMRIGHT);

    int i = 0;


    INTEnableSystemMultiVectoredInt();

    if (Tape_Init() == SUCCESS) {
        printf("\nSuccesfully initialized tape sensors");
    }
    else {
        printf("\nFailed to initialize tape sensors");
        return 1;
    }

    // Test routine (above goes in init)
    printf("\nHello tester!");
    DELAY();

    #ifndef TAPE_TEST_ALL
    unsigned int index = TAPE_LEFT_I;
    char keyPressed;
    i = 0;
    #endif

    while(1) {
        Tape_HandleSM();

        #ifdef TAPE_TEST_ALL
        printf("\n left TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_LeftTriggered(),
            ReadADPin(sensorPortMap[TAPE_LEFT_I]),
            sensorOffReadings[TAPE_LEFT_I],
            sensorOnReadings[TAPE_LEFT_I], sensorReading[TAPE_LEFT_I]);
        printf("\n center TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_CenterTriggered(),
            ReadADPin(sensorPortMap[TAPE_CENTER_I]),
            sensorOffReadings[TAPE_CENTER_I],
            sensorOnReadings[TAPE_CENTER_I], sensorReading[TAPE_CENTER_I]);
        printf("\n right TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_RightTriggered(),
            ReadADPin(sensorPortMap[TAPE_RIGHT_I]),
            sensorOffReadings[TAPE_RIGHT_I],
            sensorOnReadings[TAPE_RIGHT_I], sensorReading[TAPE_RIGHT_I]);
        printf("\n back TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_BackTriggered(),
            ReadADPin(sensorPortMap[TAPE_BACK_I]),
            sensorOffReadings[TAPE_BACK_I],
            sensorOnReadings[TAPE_BACK_I], sensorReading[TAPE_BACK_I]);
        printf("\n armleft TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_ArmLeftTriggered(),
            ReadADPin(sensorPortMap[TAPE_ARMLEFT_I]),
            sensorOffReadings[TAPE_ARMLEFT_I],
            sensorOnReadings[TAPE_ARMLEFT_I], sensorReading[TAPE_ARMLEFT_I]);
        printf("\n armright TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_ArmRightTriggered(),
            ReadADPin(sensorPortMap[TAPE_ARMRIGHT_I]),
            sensorOffReadings[TAPE_ARMRIGHT_I],
            sensorOnReadings[TAPE_ARMRIGHT_I], sensorReading[TAPE_ARMRIGHT_I]);
        printf("\n armfront TRIG=%x, ADC=%x, OFF=%x, ON=%x, READ=%x", Tape_ArmFrontTriggered(),
            ReadADPin(sensorPortMap[TAPE_ARMFRONT_I]),
            sensorOffReadings[TAPE_ARMFRONT_I],
            sensorOnReadings[TAPE_ARMFRONT_I], sensorReading[TAPE_ARMFRONT_I]);

#else
        /*
        if (i >= 10000) {
            char trig = 0;
            switch (index) {
                case 0:
                    trig = Tape_LeftTriggered();
                    break;
                case 1:
                    trig = Tape_CenterTriggered();
                    break;
                case 2:
                    trig = Tape_RightTriggered();
                    break;
                case 3:
                    trig = Tape_BackTriggered();
                    break;
                case 4:
                    trig = Tape_ArmFrontTriggered();
                    break;
                case 5:
                    trig = Tape_ArmLeftTriggered();
                    break;
                case 6:
                    trig = Tape_ArmRightTriggered();
                    break;
            }
           printf("\nSensor #%u: TAPE=%x ADC=%x, LED_OFF=%x, LED_ON=%x, AVG=%x",
                index, trig,
                ReadADPin(sensorPortMap[index]), sensorOffReadings[index],
                sensorOnReadings[index], sensorReading[index]);
            i = 0;
        }
        i++;

        keyPressed = GetChar();

        if (keyPressed != 0) {
            if (keyPressed == 'i') {
                printf("\nSelect a sensor:");
                printf("\n0=LEFT, 1=CENTER, 2=RIGHT, 3=BACK, 4=ARMFRONT, 5=ARMLEFT, 6=ARMRIGHT");
                keyPressed = GetChar();
                while(keyPressed == 0) {
                    keyPressed = GetChar();
                }
                char key[2];
                key[0] = keyPressed;
                key[1] = '\0';
                printf("\nTrying %x",key[0]);
                if (keyPressed <= 54 && keyPressed >= 48)
                    index = atoi(key);
                else
                    printf("\nInvalid sensor selected");
             }
        }
         */
#endif

            /*
            printf("\nStates: L=(%x), C=(%x), R=(%x), B=(%x), AF=(%x), AL=(%x), AR=(%x)",
               LeftReading(), CenterReading(),
               RightReading(), BackReading(),
               ArmFrontReading(), ArmLeftReading(),
               ArmRightReading());

            //printf("\nState=%d, time=%d, leds=%x", tapeState, uSecondsLeftToGo,ledsOn);
               
            i = 0;
             */
            while (!IsTransmitEmpty()); // bad, this is blocking code
    } // end of loop

    Tape_End();
    return 0;
} // test harness
#endif


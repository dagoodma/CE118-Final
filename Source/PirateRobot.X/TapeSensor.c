/*
 * File:   TapeSensor.c
 * Author: dagoodma
 *
 * Taking 3 samples, at 
 *
 * Created on February 25, 2012, 6:23 PM
 */
/*
 * File:   Geneva.c
 * Author: dagoodma
 *
 * Created on February 21, 2012, 7:55 PM
 */
#define GENEVA_TEST

#include <p32xxxx.h>
#include "serial.h"
#include "timers.h"
#include "PORTS.h"
#include "pwm.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define TAPESENSORCOUNT 7
#define TAPEPINCOUNT TAPESENSORCOUNT * 2



//--------------- Photodetectors --------------
// TD1 (top middle on TR)
#define TAPE_TRISD1 PORTV03_TRIS
#define TAPE_LATD1   PORTV03_LAT
// TD2 (bottom left on TR)
#define TAPE_TRISD2 PORTV04_TRIS
#define TAPE_LATD2 PORTV04_LAT
// TD3 (bottom right on TR)
#define TAPE_TRISD3 PORTV05_TRIS
#define TAPE_LATD3 PORTV05_LAT

//----------------- Emitters ------------------
// TE1 (top middle on TR)
#define TAPE_TRISE1 PORTZ08_TRIS
#define TAPE_LATE1   PORTZ08_LAT
// TE2 (bottom left on TR)
#define TAPE_TRISE2 PORTZ08_TRIS
#define TAPE_LATE2 PORTZ08_LAT
// TE3 (bottom right on TR)
#define TAPE_TRISE3 PORTZ09_TRIS
#define TAPE_LATE3 PORTZ09_LAT




static char TapepinMap[TAPEPINCOUNT];


/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/

static volatile unsigned int * const RC_LATSET[] = {&LATFSET, &LATBSET, &LATDSET,
    &LATESET, &LATDSET, &LATESET,
    &LATBSET, &LATBSET, &LATBSET,
    &LATBSET};
static volatile unsigned int * const RC_LATCLR[] = {&LATFCLR, &LATBCLR, &LATDCLR,
    &LATECLR, &LATDCLR, &LATECLR,
    &LATBCLR, &LATBCLR, &LATBCLR,
    &LATBCLR};

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * Function: Tape_Init
 * @param [TapePins], optionally enables only the given
 * @return SUCCESS or ERROR
 * @remark
 */
char Tape_Init(unsigned short int TapePins) {

    dbprintf("\nGeneva loaded (%d)", LOAD_WIDTH);
    return SUCCESS;
}

char Tape_Event() {

    if 

    RC_SetPulseTime(SERVO, RELEASE_WIDTH);
    dbprintf("\nGeneva released (%d)", RELEASE_WIDTH);
    return SUCCESS;
}

/*******************************************************************************
 * TEST HARNESS                                                                *
 ******************************************************************************/
#ifdef GENEVA_TEST


#ifdef GENEVA_TEST_POT
#include <AD.h>

#define POTENTIOMETER AD_PORTV5
// ---------------- Prototypes ------------------
unsigned int ReadPotentiometer(void);
unsigned int min(unsigned int a, unsigned int b);
unsigned int max(unsigned int a, unsigned int b);

#endif

#define DELAY() for(i=0;i < NOPCOUNT; i++) __asm("nop")
#define NOPCOUNT 990000

int main(void) {
    SERIAL_Init();
    #ifdef GENEVA_TEST_POT
    AD_Init(POTENTIOMETER);
    #endif
    int i = 0;
    //AD1PCFG = 0xFF;

    INTEnableSystemMultiVectoredInt();

    if (RC_Init(SERVO) == SUCCESS) {
        printf("Succesfully initialized servo (%x)", SERVO);
    }
    else {
        printf("Failed to initialize servo (%x)", SERVO);
    }

    // Test routine (above goes in init)
    printf("\nHello World!");
    DELAY();

    while(1) {
        #ifndef GENEVA_TEST_POT
        Geneva_Load();
        DELAY();
        Geneva_Release();
        DELAY();
        #else
        // Read and print potentiometer
        unsigned int potValue = ReadPotentiometer();
        //printf("\nPot. reading: %x", potValue);
        DELAY();

        unsigned int newWidth = potValue +1000;
        printf("\nPot value to %u", potValue);

        // bound it
        newWidth = max(newWidth,MINPULSE);
        newWidth = min(newWidth,MAXPULSE);
        // effect the motor
        if (RC_SetPulseTime(RC_PORT, newWidth) == SUCCESS) {
            printf("\nSuccessfully set PWM to %u", newWidth);
        }
        else {
            printf("\nFailed to set PWM to %u", newWidth);
        }
        #endif

        while (!IsTransmitEmpty()); // bad, this is blocking code
    } // end of loop

    RC_End();
    return 0;
}

#ifdef GENEVA_TEST_POT
/**
 * Function: min
 * @param a, integer a
 * @param b, integer b
 * @return Lowest value, either a or b
 * @remark
 */
unsigned int min(unsigned int a, unsigned int b) {
    if (a < b)
        return a;
    else
        return b;
}

/**
 * Function: max
 * @param a, integer a
 * @param b, integer b
 * @return Highest value, either a or b
 * @remark
 */
unsigned int max(unsigned int a, unsigned int b) {
    if (a > b)
        return a;
    else
        return b;
}



/**
* Function: ReadPotentiometer
* @return Voltage at the wiper of the potentiometer
* @remark Voltage is sampled from the ADC where 0V -> 0, and
* 3.3V -> 1023
*/
unsigned int ReadPotentiometer(void) {
    return ReadADPin(POTENTIOMETER);
}

#endif
#endif


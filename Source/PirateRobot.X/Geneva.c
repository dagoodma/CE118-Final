/*
 * File:   Geneva.c
 * Author: dagoodma
 *
 * Created on February 21, 2012, 7:55 PM
 */
#define GENEVA_TEST

#include <p32xxxx.h>
#include "RCServo.h"
#include "serial.h"
#include "timers.h"
#include "PORTS.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define MINPULSE 1000
#define MAXPULSE 2000

//#define SERVO_LAT TRISDbits.TRISD10
#define SERVO RC_PORTY06

// Positions for loading and releasing
#define LOAD_WIDTH 1000
#define RELEASE_WIDTH 1600

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * Function: Geneva_Load
 * @return SUCCESS or ERROR
 * @remark Sets the geneva wheel to the loading position
 */
char Geneva_Load() {

    RC_SetPulseTime(SERVO, LOAD_WIDTH);
    dbprintf("\nGeneva loaded (%d)", LOAD_WIDTH);
    return SUCCESS;
}

/**
 * Function: Geneva_Release
 * @return SUCCESS or ERROR
 * @remark Sets the geneva wheel to the release position
 */
char Geneva_Release() {

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


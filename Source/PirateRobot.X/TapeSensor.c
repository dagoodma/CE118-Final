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


/* Tape sensor port configuration 
    TAPE_LEFT 0x001    ===> PortV-5
    TAPE_CENTER 0x002  ===> PortV-6
    TAPE_RIGHT 0x004   ===> PortV-7
    TAPE_BACK 0x008    ===> PortV-8
    TAPE_ARMFRONT 0x010===> PortW-3
    TAPE_ARMLEFT 0x020 ===> PortW-4
    TAPE_ARMRIGHT 0x040===> PortW-5
*/



//--------------- Photodetectors --------------
#define TAPE_LEFT_TRIS PORTV05_TRIS
#define TAPE_LEFT_LAT   PORTV05_LAT
#define TAPE_CENTER_TRIS PORTV06_TRIS
#define TAPE_CENTER_LAT PORTV06_LAT
#define TAPE_RIGHT_TRIS PORTV07_TRIS
#define TAPE_RIGHT_LAT PORTV07_LAT
#define TAPE_BACK_TRIS PORTV08_TRIS
#define TAPE_BACK_LAT PORTV08_LAT
#define TAPE_ARMFRONT_TRIS PORTW03_TRIS
#define TAPE_ARMFRONT_LAT  PORTW03_LAT
#define TAPE_ARMLEFT_TRIS   PORTW04_TRIS
#define TAPE_ARMLEFT_LAT   PORTw04_LAT
#define TAPE_ARMRIGHT_TRIS   PORTW05_TRIS
#define TAPE_ARMRIGHT_LAT   PORTw05_LAT

//----------------- Emitters ------------------
#define TAPE_LED_ARMFRONT_TRIS PORTZ07_TRIS
#define TAPE_LED_ARMFRONT_LAT PORTZ07_LAT
#define TAPE_LED_LC_TRIS PORTZ10_TRIS
#define TAPE_LED_LC_LAT PORTZ10_LAT
#define TAPE_LED_RB_TRIS PORTZ11_TRIS
#define TAPE_LED_RB_LAT PORTZ11_LAT
#define TAPE_LED_ARMFRONT_TRIS PORTZ12_TRIS
#define TAPE_LED_RB_LAT PORTZ12_LAT



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
 * @param [TapePins], optionally enables only the given pins
 * @return SUCCESS or ERROR
 * @remark
 */
char Tape_Init(unsigned short int TapePins) {

    dbprintf("\nTapesensors initialized (%d)", LOAD_WIDTH);
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


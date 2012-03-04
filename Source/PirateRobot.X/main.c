/*
 * File:   main.c
 * Author: dagoodma
 *
 * Created on February 21, 2012, 7:46 PM
 */

//#define USE_MAIN

#ifdef USE_MAIN
#include <p32xxxx.h>
#include <serial.h>
#include <PORTS.h>
#include <AD.h>
#include <pwm.h>

// Port and PWM settings
#define PWM_PORT       PWM_PORTZ06 // PWM to enable
#define PWM_FREQ       200

#define DIRECTION_TRIS PORTZ04_TRIS
#define DIRECTION      PORTZ04_LAT

// Motor directions
#define FORWARD 0 // CW
#define REVERSE 1 // CCW

// Duty cycle bounds
#define DUTY_MAX MAX_PWM
#define DUTY_MIN MIN_PWM

#define uint unsigned int

#define ADC_PAUSE

// ---------------- Prototypes ------------------


void wait();
uint min(uint a, uint b);
uint max(uint a, uint b);


// ---------------- Entry point -----------------
int main(void) {

    // ----------------- Initialization --------------
    SERIAL_Init();
    TIMERS_Init();

    // Initialize channel A to 200 Hz from PWM
    PWM_Init(PWM_PORT, PWM_FREQ);

   
    // Initialize direction port
    DIRECTION_TRIS = 0;
    DIRECTION = FORWARD;

    // Initialize interrupts
    INTEnableSystemMultiVectoredInt();

    // Set Ch A PWM duty cycle to 50%
    SetDutyCycle(PWM_PORT, 500);

    printf("\nHello, I am working...");

    // Infinite loop
    while (1) {

       SetDutyCycle(PWM_PORT, 800);
       wait();
       DIRECTION = REVERSE;
       wait();
       SetDutyCycle(PWM_PORT, 00);
       wait();
       DIRECTION = FORWARD;

        while (!IsTransmitEmpty()); // bad, this is blocking code
    }

    exit:
    PWM_End();
    AD_End();
    return 0;
} // end of main()

/**
* Function: wait
* @return Nothing
* @remark Waits a small period of time
*/
void wait() {
    uint wait = 0;
    for (wait = 0; wait <= 1000000; wait++)
        asm("nop");
}

/**
* Function: min
* @param a, integer a
* @param b, integer b
* @return Lowest value, either a or b
* @remark
*/
uint min(uint a, uint b) {
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
uint max(uint a, uint b) {
    if (a > b)
        return a;
    else
        return b;
}


// ---------------------- EOF ----------------------
#endif

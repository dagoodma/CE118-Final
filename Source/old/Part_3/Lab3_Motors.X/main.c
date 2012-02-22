/*
 * File:   main.c
 * Author: hahernan & dagoodma
 *
 * Created on February 4, 2012, 8:54 PM
 */

#include <p32xxxx.h>
#include <serial.h>
#include <PORTS.h>
#include <timers.h>
#include <AD.h>
#include <pwm.h>
#include <SES.H>
#include <Stepper.h>
//#include <stdio.h>
//include <LED.h>

#define NOPCOUNT 150000
#define DELAY() for(i=0; i< NOPCOUNT; i++) __asm("nop")

#define uint unsigned int

#define STEPPER_MODE HALF
#define STEPPER_SPEED 100
#define FULL 0
#define WAVE 1
#define HALF 2

//#define STEPPER_RAMPS



// ---------------- Prototypes ------------------
uint ReadPotentiometer(void);

uint min(uint a, uint b);
uint max(uint a, uint b);


int main(void) {
    int i;
    SERIAL_Init();
    AD1PCFG = 0xFF;
    INTEnableSystemMultiVectoredInt();
    //enable LED bank 1
    TRISDbits.TRISD6 = 0;
    TRISDbits.TRISD11 = 0;
    TRISDbits.TRISD3 = 0;
    TRISDbits.TRISD5 = 0;

    //enable LED bank 2
    TRISFbits.TRISF6 = 0;
    TRISGbits.TRISG7 = 0;
    TRISDbits.TRISD7 = 0;
    TRISGbits.TRISG8 = 0;

    printf("\nHello World!");
    Stepper_Init(STEPPER_SPEED);
    Stepper_SetMode(STEPPER_MODE);
    printf("\nStepping forward 420 steps");
    Stepper_SetSteps(FORWARD,220);
    while(Stepper_GetRemainingCount() > 1) {
        printf("\t%u,speed=%u",Stepper_GetRemainingCount(),Stepper_GetSpeed());
        DELAY();
#ifdef STEPPER_RAMPS
        if (Stepper_GetRemainingCount() % 5) {
            Stepper_ChangeStepRate( (Stepper_GetSpeed() / 4) + Stepper_GetSpeed() );
        }
#endif
    }
    printf("\nForward steps done");

    DELAY();

    Stepper_SetSteps(REVERSE,220);
    while (Stepper_GetRemainingCount() > 1) {
        printf("\t%u",Stepper_GetRemainingCount());
        DELAY();
    }


    Stepper_End();

    return 0;


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

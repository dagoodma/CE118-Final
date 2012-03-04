/*
 * File:   Gate.c
 * Author: dagoodma
 *
 * Created on February 21, 2012, 7:55 PM
 */
//#define Gate_TEST

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

#define SERVO RC_PORTV04

// Positions for open and close
#define CLOSE_WIDTH 1700
#define OPEN_WIDTH 1000

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

/**
 * Function: Gate_Close
 * @return SUCCESS or ERROR
 * @remark Sets the Gate wheel to the loading position
 */
char Gate_Close() {

    RC_SetPulseTime(SERVO, CLOSE_WIDTH);
    dbprintf("\nGate closed (%d)", CLOSE_WIDTH);
    return SUCCESS;
}

/**
 * Function: Gate_Open
 * @return SUCCESS or ERROR
 * @remark Sets the Gate wheel to the closed position
 */
char Gate_Open() {

    RC_SetPulseTime(SERVO, OPEN_WIDTH);
    dbprintf("\nGate released (%d)", OPEN_WIDTH);
    return SUCCESS;
}
/**
 * Function: Gate_Init
 * @return SUCCESS or ERROR
 * @remark Iniitializes the RC servo for the gate and
 * *    Sets the gate to the closed position
 */
char Gate_Init(){

    RC_Init(SERVO);
    Gate_Close();
    return SUCCESS;
}

/**
 * Function: Gate_End
 * @return SUCCESS or ERROR
 * @remark Ends use of the gate servo.
 */
char Gate_End(){

    RC_End();
    return SUCCESS;
}




/*******************************************************************************
 * TEST HARNESS                                                                *
 ******************************************************************************/
#ifdef Gate_TEST


#define DELAY() for(i=0;i < NOPCOUNT; i++) __asm("nop")
#define NOPCOUNT 990000

int main(void) {
    SERIAL_Init();
    int i;
    char keyPressed;
    INTEnableSystemMultiVectoredInt();

    if (Gate_Init() == SUCCESS) {
        printf("Succesfully initialized servo (%x)", SERVO);
    }
    else {
        printf("Failed to initialize servo (%x)", SERVO);
    }

    Gate_Close();
    printf("\nHello World!");
    DELAY();

    while(1) {
        keyPressed = GetChar();
        if (keyPressed == 'c'){
            Gate_Close();
        }
        else if (keyPressed == 'o'){
            Gate_Open();
        }

    } // end of loop

    Gate_End();
    return 0;
}

#endif

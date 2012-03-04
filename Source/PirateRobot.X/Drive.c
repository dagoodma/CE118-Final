/*
 * File:   Gate.c
 * Author: dagoodma
 *
 * Created on March 2, 2012
 */
#define DRIVE_TEST

#include <p32xxxx.h>
#include "serial.h"
#include "PORTS.h"
#include "pwm.h"
#include "Drive.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define DEBUG_VERBOSE
#ifdef DEBUG_VERBOSE
    #define dbprintf(...) printf(__VA_ARGS__)
#else
    #define dbprintf(...)
#endif

#define MOTOR_A_PWM PWM_PORTY10
#define MOTOR_A_DIR PORTY09_LAT
#define MOTOR_A_DIR_TRIS PORTY09_TRIS
#define MOTOR_B_PWM PWM_PORTY12
#define MOTOR_B_DIR PORTY11_LAT
#define MOTOR_B_DIR_TRIS PORTY11_TRIS

#define FREQUENCY_PWM 25

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

char Drive_Init(void) {
    PWM_Init(MOTOR_A_PWM |  MOTOR_B_PWM, FREQUENCY_PWM);
    Drive_Stop();

    return SUCCESS;
}

char Drive_Update(void) {

    return SUCCESS;
}

char Drive_Turn(unsigned int turnType, unsigned int turnDir, char speed) {

    return SUCCESS;
}

char Drive_Forward(char speed){
    MOTOR_A_DIR = 0;
    MOTOR_B_DIR = 0;
    SetDutyCycle(MOTOR_A_PWM, 500);
    SetDutyCycle(MOTOR_B_PWM, 500);

    return SUCCESS;
}

char Drive_Reverse(char speed){
    MOTOR_A_DIR = 1;
    MOTOR_B_DIR = 1;
    SetDutyCycle(MOTOR_A_PWM, 500);
    SetDutyCycle(MOTOR_B_PWM, 500);

    return SUCCESS;
}

char Drive_Stop(void){
    SetDutyCycle(MOTOR_A_PWM, 0);
    SetDutyCycle(MOTOR_B_PWM, 0);

    return SUCCESS;
}

/*******************************************************************************
 * TEST HARNESS                                                                *
 ******************************************************************************/
#ifdef DRIVE_TEST

#define DELAY() for(i=0;i < NOPCOUNT; i++) __asm("nop")
#define NOPCOUNT 990000

char main(){

    int i;
    SERIAL_Init();
    INTEnableSystemMultiVectoredInt();
    Drive_Init();
    printf("\nHello World");

    while(1){

        Drive_Forward(5);
        DELAY();

        Drive_Stop();
        DELAY();

        Drive_Reverse(5);
        DELAY();

    }
}

#endif
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

//#define DRIVE_TEST
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
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/
enum motor {A, B};
enum direction {FORWARD, REVERSE};
static unsigned char motorPWM[] = {MOTOR_A_PWM, MOTOR_B_PWM};

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/
static void SetDirection(int motor, int direction);

/*******************************************************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/
static void SetDirection(int motor, int direction) {
    // TODO add error checking
    // Motors are wired backwards
    if (motor == A) {
        MOTOR_A_DIR = direction;
    }
    else {
        MOTOR_B_DIR = !direction;
    }
}

static char SetSpeed(int motor, int speed) {
    // TODO add error checking
    int newSpeed = speed * 100;
    SetDutyCycle(motorPWM[motor], newSpeed);
}

/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

char Drive_Init(void) {
    MOTOR_A_DIR_TRIS = 0;
    MOTOR_B_DIR_TRIS = 0;
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
    SetDirection(A,FORWARD);
    SetDirection(B,FORWARD);

    SetSpeed(A,speed);
    SetSpeed(B,speed);

    return SUCCESS;
}

char Drive_Reverse(char speed){
    SetDirection(A,REVERSE);
    SetDirection(B,REVERSE);

    SetSpeed(A,speed);
    SetSpeed(B,speed);

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
        printf("\nFORWARD!");
        Drive_Forward(5);
        DELAY();

        Drive_Stop();
        DELAY();
        printf("\nREVERSE!");
        Drive_Reverse(5);
        DELAY();

        Drive_Stop();
        DELAY();

        printf("\nFORWARD!");
        Drive_Forward(7);
        DELAY();

        Drive_Stop();
        DELAY();
        printf("\nREVERSE!");
        Drive_Reverse(7);
        DELAY();

        Drive_Stop();
        DELAY();

        printf("\nFORWARD!");
        Drive_Forward(9);
        DELAY();

        Drive_Stop();
        DELAY();
        printf("\nREVERSE!");
        Drive_Reverse(9);
        DELAY();

        Drive_Stop();
        DELAY();

    }
}

#endif
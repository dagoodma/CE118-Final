/*
 * File:   Drive.c
 * Author: dagoodma jurjohns
 *
 * Created on March 2, 2012
 */
//#define DRIVE_TEST

#include <p32xxxx.h>
#include "serial.h"
#include "PORTS.h"
#include "pwm.h"
#include "Drive.h"
#include "AD.h"
#include "LED.h"
#include "Util.h"

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/
#define USE_BATPWM
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

#define LEFT 0
#define RIGHT 1
#define FREQUENCY_PWM 250
#define BAT_MAX 305 // (1023 * Vbat) / (10 * 3.3) => Vbat = 9.8 V
#define UPDATE_DELAY 100
#define TIMER_NUM 5


/*******************************************************************************
 * PRIVATE VARIABLES                                                           *
 ******************************************************************************/
enum motor {B, A};
enum direction {FORWARD, REVERSE};

static unsigned char motorPWM[] = {MOTOR_A_PWM, MOTOR_B_PWM};
static unsigned int motorPWMValue[] = {0, 0};
static unsigned char motorSpeed[] = { 0, 0 };

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                *
 ******************************************************************************/
static void SetDirection(int motor, int direction);

/*************************************

******************************************
 * PRIVATE FUNCTIONS                                                           *
 ******************************************************************************/
static void SetDirection(int motor, int direction) {

    if (motor == A) {
        MOTOR_A_DIR = !direction;
    }
    else {
        MOTOR_B_DIR = !direction;
    }
}

static char SetSpeed(int motor, char speed) {
    // TODO add error checking
    motorSpeed[motor] =  speed;
    //printf("\nPWM should be = %f", speed * 80.0);
 
    unsigned int pwm = 0;
    if (speed > 0) {
#ifdef USE_BATPWM
        int battery = (float)ReadADPin(BAT_VOLTAGE);
#else
        int battery = 0;
#endif
        float pwm_modifier = 100.0 * ((1.0 - 1.0*(battery/BAT_MAX)) * 2.15);
        //printf("\nMultiplier =%f", max((int)pwm_modifier, 0));
        pwm = (unsigned int)(speed * 90.0 + max((int)pwm_modifier, 0));
    }
    motorPWMValue[motor] = pwm;
    //printf("\nPWM is=%i", pwm);
    pwm = min(pwm, 1000);
    pwm = max(pwm, 0);

    SetDutyCycle(motorPWM[motor], pwm);
}

void SetMotor(int motor, int direction, char speed) {
    SetDirection(motor, direction);
    SetSpeed(motor, speed);
}


/*******************************************************************************
 * PUBLIC FUNCTIONS                                                           *
 ******************************************************************************/

char Drive_Init(void) {
    // Set motor direction outputs
    MOTOR_A_DIR_TRIS = 0;
    MOTOR_B_DIR_TRIS = 0;

    PWM_Init(MOTOR_A_PWM |  MOTOR_B_PWM, FREQUENCY_PWM);
    Drive_Stop();
    InitTimer(TIMER_NUM,UPDATE_DELAY);

    return SUCCESS;
}

char Drive_Update(void) {
    if (IsTimerExpired(TIMER_NUM)) {
        SetSpeed(A, motorSpeed[A]);
        SetSpeed(B, motorSpeed[B]);
        InitTimer(TIMER_NUM, UPDATE_DELAY);
    }
    return SUCCESS;
}

char Drive_Turn(enum turnType type, enum turnDir dir, char speed) {
    static char previousTurn;
    switch(type){
        case pivot:
            switch(dir){
                case right:
                    SetMotor(A, REVERSE, speed);
                    SetMotor(B, FORWARD, speed);
                    previousTurn = RIGHT;
                    break;
                case left:
                    SetMotor(A, FORWARD, speed);
                    SetMotor(B, REVERSE, speed);
                    previousTurn = LEFT;
                    break;
                case same:
                    if(previousTurn == LEFT)
                        Drive_Turn(pivot, left, speed);
                    else if(previousTurn == RIGHT)
                        Drive_Turn(pivot, right, speed);
                    break;
            case opposite:	
                    if(previousTurn == LEFT)
                        Drive_Turn(pivot, right, speed);
                    else if(previousTurn == RIGHT)
                        Drive_Turn(pivot, left, speed);
                    break;
            }
            break;
        case rightAng:			//leaving for now, should prob. be a timed pivot and only take L/R
            switch(dir){
                case right:
                    previousTurn = RIGHT;
                    break;
                case left:
                    previousTurn = LEFT;
                    break;
                case same:
                    if(previousTurn == LEFT)
                        Drive_Turn(rightAng, left, speed);
                    else if(previousTurn == RIGHT)
                        Drive_Turn(rightAng, right, speed);
                    break;
                case opposite:
                    if(previousTurn == LEFT)
                        Drive_Turn(rightAng, right, speed);
                    else if(previousTurn == RIGHT)
                        Drive_Turn(rightAng, left, speed);
                    break;
            }
            break;
        case soft:	
            switch(dir){
                case right:
                    SetMotor(A, FORWARD, speed/2);
                    SetMotor(B, FORWARD, speed);

                    previousTurn = RIGHT;
                    break;
                case left:
                    SetMotor(A, FORWARD, speed);
                    SetMotor(B, FORWARD, speed/2);

                    previousTurn = LEFT;
                    break;
                case same:
                    if(previousTurn == LEFT)
                        Drive_Turn(soft, left, speed);
                    else if(previousTurn == RIGHT)
                        Drive_Turn(soft, right, speed);
                    break;
                case opposite:
                      if(previousTurn == LEFT)
                        Drive_Turn(soft, right, speed);
                      else if(previousTurn == RIGHT)
                        Drive_Turn(soft, left, speed);
                    break;
            }
            break;
        case hard:
            switch(dir){
                case right:
                    SetMotor(A, FORWARD, speed);
                    SetMotor(B, FORWARD, speed/3);
                    previousTurn = RIGHT;
                    break;
                case left:
                    SetMotor(A, FORWARD, speed/3);
                    SetMotor(B, FORWARD, speed);
                    previousTurn = LEFT;
                    break;
                case same:
                     if(previousTurn == LEFT)
                        Drive_Turn(hard, left, speed);
                    else if(previousTurn == RIGHT)
                        Drive_Turn(hard, right, speed);
                     break;
                case opposite:
                     if(previousTurn == LEFT)
                        Drive_Turn(hard, right, speed);
                      else if(previousTurn == RIGHT)
                        Drive_Turn(hard, left, speed);
                    break;
            }
            break;
        default:
            break;
    }
    return SUCCESS;
}

char Drive_Forward(char speed){
    SetMotor(A, FORWARD, speed);
    SetMotor(B, FORWARD, speed);
    return SUCCESS;
}

char Drive_Reverse(char speed){
    SetMotor(A, REVERSE, speed);
    SetMotor(B, REVERSE, speed);

    return SUCCESS;
}

char Drive_Stop(void) {
    SetMotor(A, FORWARD, 0);
    SetMotor(B, FORWARD, 0);

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
    int j;
    SERIAL_Init();
    INTEnableSystemMultiVectoredInt();
    Drive_Init();
    dbprintf("\nHello World");
    AD_Init(BAT_VOLTAGE);
    LED_Init(LED_BANK3);
    LED_OffBank(LED_BANK3, 0xf);
    DELAY();
    int pwm;

    while(1) {
        printf("\n batt voltage: %d", ReadADPin(BAT_VOLTAGE));
       Drive_Stop(); DELAY();

       for (j = 10; j >= 0; j--) {
        Drive_Forward(j);
        printf("\nFORWARD! speed=%u pwm=%u", motorSpeed[A], motorPWMValue[A]);
        DELAY(); Drive_Stop(); DELAY();
       }
        /*
        Drive_Forward(10);
        DELAY();
        pwm = 80 * 10 + (200.0*(1.0 - 1.0*((float)ReadADPin(BAT_VOLTAGE)/(BAT_MAX))) * 2.15);
        printf("\n 10 speed pwm  set to:  %d",pwm);
        DELAY();
        printf("\nFORWARD!");
        Drive_Forward(8);
        DELAY();
        pwm = 80 * 8 + (200.0*(1.0 - 1.0*((float)ReadADPin(BAT_VOLTAGE)/(BAT_MAX))) * 2.15);
        printf("\n 8 speed pwm  set to:  %d",pwm);
        DELAY();
        printf("\nFORWARD!");
        Drive_Forward(3);
        DELAY();
        pwm = 80 * 3 + (200.0*(1.0 - 1.0*((float)ReadADPin(BAT_VOLTAGE)/(BAT_MAX))) * 2.15);
        printf("\n 3 speed pwm  set to:  %d",pwm);
        Drive_Stop();
        DELAY();

        printf("\nhard right !");
        Drive_Turn(hard, right, 7);
        DELAY();
        pwm = 7 * 80.0 + (200.0*(1.0 - 1.0*((float)ReadADPin(BAT_VOLTAGE)/(BAT_MAX))) * 2.15);
         DELAY();
        printf("\n 7 speed pwm  set to:  %d",pwm);
        DELAY();
        DELAY();

        Drive_Stop();
        DELAY();
        printf("\nsoft right!");
        Drive_Turn(hard, opposite, 10);
        pwm = 10 * 80.0 + (200.0*(1.0 - 1.0*((float)ReadADPin(BAT_VOLTAGE)/(BAT_MAX))) * 2.15);
        DELAY();
        printf("\n 10 speed pwm  set to:  %d",pwm);
        DELAY();
        DELAY();
        DELAY();
        Drive_Stop();
        DELAY();
        /*

      printf("\nPivot Left!");
        Drive_Turn(soft, left, 9);
        DELAY();
        DELAY();
        DELAY();

        Drive_Stop();
        DELAY();
        printf("\nPIVOT opposite!");
        Drive_Turn(pivot, right, 9);
        DELAY();
        DELAY();
        DELAY();
        Drive_Stop();
        DELAY();
        /*
        printf("\nPIVOT right!");
        Drive_Turn(pivot, right, 9);
		Drive_Stop();
        DELAY();
        printf("\nPIVOT opposite!");
        Drive_Turn(pivot, opposite, 9);
        DELAY();
        DELAY();
        DELAY();


*/    }
}

#endif

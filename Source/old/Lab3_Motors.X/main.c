/*
 * File:   main.c
 * Author: hahernan & dagoodma
 *
 * Created on February 4, 2012, 8:54 PM
 */

#include <p32xxxx.h>
#include <serial.h>
#include <PORTS.h>
#include <AD.h>
#include <pwm.h>
#include <RCServo.h>

#define RC_PORT RC_PORTV04

#define DIRECTION_TRIS       PORTZ03_TRIS
#define  DIRECTION           PORTZ03_LAT

#define FORWARD 0 // CW
#define REVERSE 1 // CCW

#define MINPULSE 1000
#define MAXPULSE 2000

#define POT_INPUT AD_PORTV5
#define ADC_PAUSE

// ---------------- Prototypes ------------------
unsigned int min(unsigned int a, unsigned int b);
unsigned int max(unsigned int a, unsigned int b);
unsigned int ReadPotentiometer(void);

int main(void) {

    // ----------------- Initialization --------------
    SERIAL_Init();
    AD_Init(POT_INPUT);

    // Initialize interrupts
    INTEnableSystemMultiVectoredInt();

    RC_Init(RC_PORT);
    unsigned int wait = 0;
    for (wait = 0; wait <= 1000000; wait++)
        asm("nop");
    RC_SetPulseTime(RC_PORT, 2000);

    printf("\nHello,...");

    while (1) {
        
        // Read and print potentiometer
        unsigned int potValue = ReadPotentiometer();
        //printf("\nPot. reading: %x", potValue);

        // Pause if desired
        #ifdef ADC_PAUSE
        
        #endif

        unsigned int newSpeed = potValue +1000;
        printf("\nPot value to %u", potValue);

        // bound it
        newSpeed = max(newSpeed,MINPULSE);
        newSpeed = min(newSpeed,MAXPULSE);
        // effect the motor
        if (RC_SetPulseTime(RC_PORT, newSpeed) == SUCCESS) {
            printf("\nSuccessfully set PWM to %u", newSpeed);
        }
        else {
            printf("\nFailed to set PWM to %u", newSpeed);
        }
         
/*
        char keyPressed = GetChar();
        if (keyPressed != 0) {
            /*
            if (keyPressed == 'f') {
                // forward
                printf("\nforward");
                if (DIRECTION != FORWARD)
                    DIRECTION = FORWARD;
            }
            else if(keyPressed == 'r') {
                 // reverse
                printf("\nreverse");
                if (DIRECTION != REVERSE)
                    DIRECTION = REVERSE;
            }
            else if(keyPressed == 'q') {
                printf("\nGoodbye!");
                PWM_End();
                return 0;
            }
             
            if(keyPressed == 't') {
                unsigned short int stringMax = 4;
                unsigned short int i = 0;
                char charNumber[1];
                charNumber[0] = 0;
                char stringNumber[stringMax+1];
                printf("\nPlease enter a frequency:");

                while(charNumber[0] != 46) {
                        charNumber[0] = GetChar();
                        //printf("\nGot %s", charNumber);
                        if ((charNumber[0] <= 57 && charNumber[0] >= 48)) {
                            stringNumber[i] = charNumber[0];
                            stringNumber[i+1] = '\0';
                            //printf("\nAppended %c to '%s'",charNumber[0],stringNumber);
                            //stringNumber = strcat(stringNumber, charNumber);
                            i++;
                        }

                        if (i >= stringMax)
                            break;
                        
                }
                //printf("\ni=%u stmax=%u", i, stringMax);
                printf("\nDone");

                unsigned int newFreq = atoi(stringNumber);
                newFreq = max(newFreq,MINPULSE);
                newFreq = min(newFreq,MAXPULSE);
                printf("\nPulse width set to %u",newFreq);
                RC_SetPulseTime(RC_PORT,newFreq);
                
            }
        } // end of keyPressed
        */
 
        while (!IsTransmitEmpty()); // bad, this is blocking code
    } // end of while loop

    return 0;
}

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
    return ReadADPin(POT_INPUT);
}

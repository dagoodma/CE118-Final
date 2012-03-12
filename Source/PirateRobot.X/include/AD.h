
/*
 * File:   AD.h
 * Author: mdunne
 *
 * Created on November 22, 2011, 8:57 AM
 */

#ifndef AD_H
#define AD_H

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
#ifndef SUCCESS
#define SUCCESS 0
#define ERROR -1
#endif

#define AD_PORTV3 (1<<0)
#define AD_PORTV4 (1<<1)
#define AD_PORTV5 (1<<2)
#define AD_PORTV6 (1<<3)

#define AD_PORTV7 (1<<4)
#define AD_PORTV8 (1<<5)
#define AD_PORTW3 (1<<6)
#define AD_PORTW4 (1<<7)
#define AD_PORTW5 (1<<8)
#define AD_PORTW6 (1<<9)
#define AD_PORTW7 (1<<10)
#define AD_PORTW8 (1<<11)
#define BAT_VOLTAGE (1<<12)


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * Function: AD_Init
 * @param Pins, used #defined AD_PORTxxx OR'd together for each A/D Pin
 * @return SUCCESS or ERROR
 * @remark Initializes the A/D pins requested into analog inputs and configures the A/D subsystem.
 * It then generates the mapping for correctly reading the pin and then starts the A/D system.
 * @author Max Dunne
 * @date 2011.12.10  */
unsigned char AD_Init(unsigned int Pins);


/**
 * Function: ReadADPin
 * @param Pin, used #defined AD_PORTxxx to select pin
 * @return 10-bit AD Value or ERROR
 * @remark Reads current value from buffer for given pin
 * @author Max Dunne
 * @date 2011.12.10  */
unsigned int ReadADPin(unsigned int Pin);



/**
 * Function: AD_End
 * @param None
 * @return None
 * @remark disables the A/D subsystem and release the pins used
 * @author Max Dunne
 * @date 2011.12.10  */
void AD_End(void);

#endif

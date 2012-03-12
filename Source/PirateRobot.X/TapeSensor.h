/*
 * File:   TapeSensor.h
 * Author: dagoodma, hahernan, jurjohns
 *
 * Public interface for the TapeSensor module. Tape sensors ports are
 * initialized externally by the caller using AD_Init.
 *
 * Created on February 25, 2012, 7:50 PM
 */

#ifndef TapeSensor_H
#define TapeSensor_H

#include "PORTS.h"

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/

#ifndef SUCCESS
#define SUCCESS 0
#define ERROR -1
#endif

#ifndef TRUE
#define FALSE 0
#define TRUE 1
#endif

enum sensorIndex { TAPE_LEFT_I, TAPE_CENTER_I, TAPE_RIGHT_I, TAPE_BACK_I,
    TAPE_ARMFRONT_I, TAPE_ARMLEFT_I, TAPE_ARMRIGHT_I };


//--------------- Photodetectors --------------
#define TAPE_LEFT   AD_PORTV5
#define TAPE_CENTER AD_PORTV6
#define TAPE_RIGHT AD_PORTV7
#define TAPE_BACK AD_PORTV8
#define TAPE_ARMFRONT AD_PORTW3
#define TAPE_ARMLEFT AD_PORTW4
#define TAPE_ARMRIGHT AD_PORTW5


/*
#define TAPE_LEFT 0x001      // PortV-5
#define TAPE_CENTER 0x002    // PortV-6
#define TAPE_RIGHT 0x004     // PortV-7
#define TAPE_BACK 0x008      // PortV-8
#define TAPE_ARMFRONT 0x010  // PortW-3
#define TAPE_ARMLEFT 0x020   // PortW-4
#define TAPE_ARMRIGHT 0x040  // PortW-5
*/

/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/


/**
 * @Function: Tape_Init
 * @return SUCCESS or ERROR
 * @remark Initializes the tape sensors.
 * @date 2012.2.28 08:34 */
char Tape_Init();


/**
 * @Function: Tape_HandleSM
 * @return SUCCESS or ERROR
 * @remark Handles the tape sensor's state machine. This should be
 *         called before handling any other state machines.
 * @date 2012.2.27 05:42 */
char Tape_HandleSM();


/**
 * @Function: Tape_End
 * @return SUCCESS or ERROR
 * @remark Used to end use of the tape sensors.
 * @date 2012.2.29 12:59 */
char Tape_End();


// ********************* Tape Sensor Accessors *************************
/**
 * @Function: Tape_LeftTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the left tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_LeftTriggered();

/**
 * @Function: Tape_CenterTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the center tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_CenterTriggered();

/**
 * @Function: Tape_RightTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the right tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_RightTriggered();

/**
 * @Function: Tape_BackTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the back tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_BackTriggered();

/**
 * @Function: Tape_ArmFrontTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the arm's front tape sensor is
 *         triggered.
 * @date 2012.2.27 05:42 */
char Tape_ArmFrontTriggered();

/**
 * @Function: Tape_ArmLeftTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the arm's left tape sensor is
 *         triggered.
 * @date 2012.2.27 05:42 */
char Tape_ArmLeftTriggered();

/**
 * @Function: Tape_ArmRightTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the arm's right tape sensor is
 *         triggered.
 * @date 2012.2.27 05:42 */
char Tape_ArmRightTriggered();

/**
 * @Function: Tape_AnyTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if any tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_AnyTriggered();

/**
 * @Function: Tape_AnyRightTriggered
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if right tape sensor or any arm sensor
 *  triggered is triggered.
 * @date */
char Tape_AnyRightTriggered();


void Tape_SetOnTapeThreshold(unsigned int index);
void Tape_SetOffTapeThreshold(unsigned int index);

#endif

/*
 * File:   TapeSensor.h
 * Author: dagoodma, hahernan, jurjohns
 *
 * Public interface for the TapeSensor module. Tape sensors are
 * initialized, and then the event checker and servicer functions
 * (Tape_Event and Tape_Service) are registered externally by the
 * caller.
 *
 * Created on February 25, 2012, 7:50 PM
 */

#ifndef TapeSensor_H
#define TapeSensor_H

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


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/



/**
 * @Function: IR_Init
 * @return SUCCESS or ERROR
 * @remark Initializes the IR sensor module's state machine
 * @date 2012.3.5 13:09 */
char IR_Init();


/**
 * @Function: IR_HandleSM
 * @return SUCCESS or ERROR
 * @remark Advances the IR sensor state machine.
 * @date 2012.3.5 12:47 */
char IR_HandleSM();


/**
 * @Function: IR_MainTriggered
 * @return TRUE or FALSE
 * @remark Whether the IR main detector sees a beacon
 * @date 2012.3.5 12:47 */
char IR_MainTriggered();


/**
 * @Function: IR_AngleTriggered
 * @return TRUE or FALSE
 * @remark Whether the IR angle detector sees a beacon
 * @date 2012.3.5 12:47 */
char IR_AngleTriggered();


/**
 * @Function: IR_End
 * @return SUCCESS or ERROR
 * @remark Ends the IR sensor module's state machine
 *         the appropriate select port.
 * @date 2012.3.5 13:10 */
char IR_End();

#endif


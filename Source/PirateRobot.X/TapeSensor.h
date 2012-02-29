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

#define TAPE_LEFT 0x001      // PortV-5
#define TAPE_CENTER 0x002    // PortV-6
#define TAPE_RIGHT 0x004     // PortV-7
#define TAPE_BACK 0x008      // PortV-8
#define TAPE_ARMFRONT 0x010  // PortW-3
#define TAPE_ARMLEFT 0x020   // PortW-4
#define TAPE_ARMRIGHT 0x040  // PortW-5

/**
 * @Function: Tape_Init()
 * @return SUCCESS or ERROR
 * @remark Initializes the tape sensors.
 * @date 2012.2.28 08:34 */
char Tape_Init();


/**
 * @Function: Tape_HandleSM()
 * @return SUCCESS or ERROR
 * @remark Handles the tape sensor's state machine. This should be
 *         called before handling any other state machines.
 * @date 2012.2.27 05:42 */
char Tape_HandleSM();

// ********************* Tape Sensor Accessors *************************
/**
 * @Function: Tape_LeftTriggered()
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the left tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_LeftTriggered();

/**
 * @Function: Tape_CenterTriggered()
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the center tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_CenterTriggered();

/**
 * @Function: Tape_RightTriggered()
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the right tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_RightTriggered();

/**
 * @Function: Tape_BackTriggered()
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the back tape sensor is triggered.
 * @date 2012.2.27 05:42 */
char Tape_BackTriggered();

/**
 * @Function: Tape_ArmFrontTriggered()
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the arm's front tape sensor is
 *         triggered.
 * @date 2012.2.27 05:42 */
char Tape_ArmFrontTriggered();

/**
 * @Function: Tape_ArmLeftTriggered()
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the arm's left tape sensor is
 *         triggered.
 * @date 2012.2.27 05:42 */
char Tape_ArmLeftTriggered();

/**
 * @Function: Tape_ArmRightTriggered()
 * @return TRUE or FALSE value
 * @remark Returns a TRUE value if the arm's right tape sensor is
 *         triggered.
 * @date 2012.2.27 05:42 */
char Tape_ArmRightTriggered();

#endif

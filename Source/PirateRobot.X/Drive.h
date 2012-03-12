/*
 * File:   Drive.h
 * Author: dagoodma, hahernan, jurjohns
 *
 * Public interface for the Drive module.
 *
 * Created on February 25, 2012, 7:50 PM
 */

#ifndef Drive_H
#define Drive_H

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

#define HALF_SPEED 5
#define FULL_SPEED 10
#define MID_SPEED 7
#define MIN_SPEED 3

enum turnType{pivot, rightAng, soft, hard};
enum turnDir{right, left, same, opposite};


/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * Function: Drive_Init()
 * @return SUCCESS or ERROR
 * @remark Initializes the two motors for driving
 */
char Drive_Init(void);

/**
 * @Function: Drive_Update()
 * @return SUCCESS or ERROR
 * @remark Updates the battery and adjusts the speed accordingly
 * @date  */
char Drive_Update(void);


/**
 * @Function: Drive_Turn()
 * @param Type of turn to be made {pivot, rightAng, soft, hard}
 * @param Direction to make the turn {right, left, same, opposite}
 * @param Speed of turn 0(slowest) to 10(fastest)
 * @return SUCCESS or ERROR
 * @remark Changes speed of motors to make a turn
 * @date  */
char Drive_Turn(unsigned int turnType, unsigned int turnDir, char speed);


/**
 * @Function: Drive_Forward()
 * @param speed of driving forward 0(slowest) 10(fastest)
 * @return SUCCESS or ERROR
 * @remark Drives the motors straight and forward
 * @date  */
char Drive_Forward(char speed);

/**
 * @Function: Drive_Reverse()
 * @param speed of driving forward 0(slowest) 10(fastest)
 * @return TRUE or FALSE value
 * @remark Drives the motors straight and reverse
 * @date */
char Drive_Reverse(char speed);

/**
 * @Function: Drive_Stop()
 * @return TRUE or FALSE value
 * @remark Stops the motors
 * @date */
char Drive_Stop(void);

#endif

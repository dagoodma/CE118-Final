/*
 * File:   Gate.h
 * Author: jurjohns
 *
 * Created on Feburary 27, 2012
 */

#ifndef Gate_H
#define Gate_H

/*******************************************************************************
 * PUBLIC #DEFINES                                                             *
 ******************************************************************************/
#ifndef SUCCESS
#define SUCCESS 0
#define ERROR -1
#endif



/*******************************************************************************
 * PUBLIC FUNCTION PROTOTYPES                                                  *
 ******************************************************************************/

/**
 * Function: Gate_Close()
 * @param none
 * @return SUCCESS or ERROR
 * @remark Holds the gate closed
 * @author David Goodman
 * /

char Gate_Close() ;


/**
 * Function: Gate_Open
 * @param none
 * @return SUCCESS or ERROR
 * @remark Releases the gate
 * @author David Goodman
 
char Gate_Open();


/**
 * Function: Gate_Init()
 * @param None
 * @return Success or Error
 * @remark Initializes the servo pin used and closes the gate
 * @author Justin Johnson
 
char Gate_Init();

/**
 * Function: Gate_End
 * @param none
 * @return SUCCESS or ERROR
 * @remark Ends use of the gate servo.
 */
char Gate_End();

#endif

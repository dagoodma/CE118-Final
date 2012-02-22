/****************************************************************************
 Module
         sestest.c

 Revision
         1.0.0

 Description
         Test/Demonstration module for the Software Events and Services
         Framework.

 Notes

 History
 When           Who What/Why
 -------------- --- --------
 04/07/05       rjk  updated to compile on CW12 for S12C32
 
 06/06/00 16:45 rmo  updated to compile under ICC11 v5.0, and work with the
                     CockRoaches and library modules in use in ME118, W00.
 01/12/98 14:35 jec  made changes to add functions from MiniTime module for
                     timing.
 04/12/94 08:26 jec  made changes to add detecting and reporting of lightON
                     and lightOff events.
                     Also hide all the details of passing a parameter
                     between an event and a service routine.
 04/11/94 09:05 jec  cleaned up after a porting from Adams initial code.
****************************************************************************/

/*----------------------------- Include Files -----------------------------*/
#include <stdio.h>
#include <stdlib.h> /* needed for rand() */
#include <roach.h>
#include <ses.h>
#include <serial.h>
#include <timers.h>
/*----------------------------- Module Defines ----------------------------*/
#define HI_LIGHT_THRESHOLD 80
#define LO_LIGHT_THRESHOLD 60
#define ONE_SEC 1000
#define TIME_INTERVAL (ONE_SEC*3)

/*------------------------------ Module Types -----------------------------*/

/*---------------------------- Global Functions ---------------------------*/

/*---------------------------- Module Functions ---------------------------*/



unsigned char getKey(EVENT_PARAM);
void putKey(SERVICE_PARAM); 
unsigned char TestFor1050(EVENT_PARAM);
void RespFor1050(SERVICE_PARAM);
unsigned char TestForLightOn(EVENT_PARAM);
void RespForLightOn(SERVICE_PARAM);
unsigned char TestForLightOff(EVENT_PARAM);
void RespForLightOff(SERVICE_PARAM);
char NewTime(void);
unsigned char TimeOut(EVENT_PARAM);
void TimeOutResp(SERVICE_PARAM);


/*---------------------------- Module Variables ---------------------------*/

/*------------------------------ Module Code ------------------------------*/


void main(void)
{
    Roach_Init();
    SES_Init(SES_ROUND_ROBIN);

    printf("starting...\n");

    SES_Register(TestFor1050,RespFor1050);
    SES_Register(TestForLightOn,RespForLightOn);
    SES_Register(TestForLightOff,RespForLightOff);
    SES_Register(TimeOut, TimeOutResp);    

	  InitTimer(0, TIME_INTERVAL);


    while(1) {  
	    SES_HandleEvents();
    }
}

/****************************************************************************
 Function
         GetKey

 Parameters
         EVENT_PARAM    standard parameter for Event checker. Can be
                        used to pass a parameter to the Service routine.

 Returns
         unsigned char  non-zero if the event was detected.

 Description
         simply checks to see if a key from the terminal is waiting. If so,
         it is placed in the shared variable and TRUE (1) is returned to
         announce that the event has occured.

 Notes
         None.

 Author
         J. Edward Carryer, 04/11/94 09:14
****************************************************************************/

char theKey;
unsigned char getKey(EVENT_PARAM)
{
    if (!IsReceiveEmpty())
    {
        theKey = getchar();
        SET_SHARED_BYTE_TO(theKey); /* leave value where the service routine */
        return 1;                   /* can find it */
    }
    else
        return 0;
}


/****************************************************************************
 Function
         putKey

 Parameters
         SERVICE_PARAM    standard service routine parameter.

 Returns
         None.

 Description
         send the value of the shared variable key to the terminal.

 Notes
         None.

 Author
         J. Edward Carryer, 04/11/94 09:16
****************************************************************************/

void putKey(SERVICE_PARAM)
{
    putchar(GET_SHARED_BYTE());
    putchar('.');
}


/****************************************************************************
 Function
         TestFor1050

 Parameters
         EVENT_PARAM.

 Returns
         1 if the test is true, false (0) otherwise.

 Description
         tests if the current time has changed since the last time through
         and that the new value is evenly divisible by 1050.

 Notes
         None.

 Author
         J. Edward Carryer, 04/11/94 09:18
****************************************************************************/
unsigned char TestFor1050(EVENT_PARAM)
{
	srand(GetTime());
    if ((rand() % 1050) == 0)
        return 1;
    else
        return 0;
}

/****************************************************************************
 Function
         RespFor1050

 Parameters
         SERVICE_PARAM  standard service function parameter. provides access
                        to any shared variables.

 Returns
         None.

 Description
         simply drives forward to indicate that the event has
         happened.

 Notes
         None.

 Author
         J. Edward Carryer, 04/11/94 09:26
****************************************************************************/
void RespFor1050(SERVICE_PARAM)
{
    RightMtrSpeed(10);
    LeftMtrSpeed(10);
    printf("1050\n");
}

/****************************************************************************
 Function
         TestForLightOff

 Parameters
         EVENT_PARAM.

 Returns
         1 if the test is true, false (0) otherwise.

 Description
         tests if the current reading from the light sensor changed since 
         the last time through, going OFF

 Notes
         Implements a hystersis band to prevent bouncing triggers.

 Author
         J. Edward Carryer, 04/11/94 09:18
****************************************************************************/
unsigned char TestForLightOff(EVENT_PARAM)
{
    static unsigned char LastLight = 0;
    static unsigned char Threshold = LO_LIGHT_THRESHOLD;

    char EventOccured = ((LightLevel() < Threshold) && (LastLight >= Threshold));

/* 
        provide for hysterisis around the switching point to eliminate
        false triggers.
*/
    if (EventOccured)
        Threshold = HI_LIGHT_THRESHOLD;
    else
        if (LightLevel() >= Threshold)
            Threshold = LO_LIGHT_THRESHOLD; 

    LastLight = LightLevel();
    return (EventOccured);
}

/****************************************************************************
 Function
         RespForLightOff

 Parameters
         SERVICE_PARAM  standard service funetion parameter. provides access
                        to any shared variables.

 Returns
         None.

 Description
         simply spins motors to indicate that the event has
         happened.

 Notes
         None.

 Author
         J. Edward Carryer, 04/11/94 09:26
****************************************************************************/
void RespForLightOff(SERVICE_PARAM)
{   
    RightMtrSpeed(5);
    LeftMtrSpeed(-5);
    printf("light off\n");

}

/****************************************************************************
 Function
         TestForLightON

 Parameters
         EVENT_PARAM.

 Returns
         1 if the test is true, false (0) otherwise.

 Description
         tests if the current reading from the light sensor changed since 
         the last time through, going ON.

 Notes
         Implements a hystersis band to prevent bouncing triggers.

 Author
         J. Edward Carryer, 04/11/94 09:18
****************************************************************************/
unsigned char TestForLightOn(EVENT_PARAM)
{
    static unsigned char LastLight = 0;
    static unsigned char Threshold = HI_LIGHT_THRESHOLD;

    char EventOccured = ((LightLevel() > Threshold) && (LastLight <= Threshold));

/* 
        provide for hysterisis around the switching point to eliminate
        false triggers.
*/
    if (EventOccured)
        Threshold = LO_LIGHT_THRESHOLD;
    else
        if (LightLevel() <= Threshold)
            Threshold = HI_LIGHT_THRESHOLD; 

    LastLight = LightLevel();
    return (EventOccured);
}

/****************************************************************************
 Function
         RespForLightOn

 Parameters
         SERVICE_PARAM  standard service funetion parameter. provides access
                        to any shared variables.

 Returns
         None.

 Description
         simply spins motors to indicate that the event has
         happened.

 Notes
         None.

 Author
         J. Edward Carryer, 04/11/94 09:26
****************************************************************************/
void RespForLightOn(SERVICE_PARAM)
{	
	  RightMtrSpeed(-5);
    LeftMtrSpeed(5);
    printf("light on\n");	
}


/****************************************************************************
 Function
         TimeOut

 Parameters
         EVENT_PARAM    standard parameter for Event checker. Can be
                        used to pass a parameter to the Service routine.

 Returns
         1 (TRUE) if timer 0 has expired. 0 (FALSE) otherwise.

 Description
         What more do you need to say ?

 Notes
         None.

 Author
         J. Edward Carryer, 02/25/97 01:32
****************************************************************************/
unsigned char TimeOut(EVENT_PARAM)
{
	return(IsTimerExpired(0));
}


/****************************************************************************
 Function
         TimeOutResp
 Parameters
         SERVICE_PARAM  standard service funetion parameter. provides access
                        to any shared variables.

 Returns
         None.

 Description
         simply stops motors to indicate 
         that the event has happened, and re-starts timer 0.

 Notes

 Author
         J. Edward Carryer, 04/11/94 09:26
****************************************************************************/
void TimeOutResp(SERVICE_PARAM)
{
  RightMtrSpeed(0);
  LeftMtrSpeed(0);
  printf("time out\n");
  
	InitTimer(0, TIME_INTERVAL);
}
/*------------------------------- Footnotes -------------------------------*/

/*------------------------------ End of file ------------------------------*/

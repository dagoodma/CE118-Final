/***********************************************************************
	FILE : ses.c
	CONTENTS : This is the module to run the software events and services
		system.  It is started with a call to SES_Init().  Events are
		registered using the SES_Register function.  The services are
		run be a call to SES_HandleEvents.  The SES_TimeToServe function
		is used to poll the timing for looping.  The system is ended
		with a call to SES_End.
	CREATED : March 3, 1994
	LINKS WITH :
	6811 RESOURCES USED :
		RTI -- If the timing is used.
	NOTES/REVISIONS :
		November 15, 1999	Matt Ohline		modified to compile under ICC11v5
***********************************************************************/

	/* === INCLUDES === */
#include <ses.h>

	/* === LOCAL DEFINITIONS === */
	/* --- The maximum number of event/service pairs --- */
#define MAX_ENTRIES	128
#define INSTALL_FLAG 1


	/* === DATA STRUCTURES === */
typedef struct e_s_pair {
	unsigned char (*event)(void *);
	void (*service)(void *);
} e_s_pair;


	/* === GLOBALS (NO EXTERNAL ACCESS) === */
static unsigned char installed;				/* flag to say if init was called */
static unsigned char entries;				/* the number of pairs registered */
static e_s_pair e_s[MAX_ENTRIES];	/* the event/service pairs */
static unsigned char scheduleType;			/* the scheduling algorithm */

	/* === PRIVATE FUNCTION PROTOTYPES === */


/***********************************************************************
	FUNCTION : SES_Init
	CONTENTS : This is the initialization routine for the SES
		functions.  It will setup the installed flag.
	PARAMETERS :
		aScheduleType -- The scheduling algorithm.
		aTimePeriod -- The timing for servicing.
	RETURNS :
		unsigned char -- Flag for success (OK_OPERATION == success).  Other returns
			ERR_BADSCHEDULE
			ERR1_BADTIMEPERIOD
	NOTES :
***********************************************************************/
unsigned char SES_Init(unsigned char aScheduleType)
{
	if (aScheduleType != SES_ROUND_ROBIN && aScheduleType != SES_PRIORITY) 
		return ERROR;

		/* set initial values */
	entries = 0;
	scheduleType = aScheduleType;
	

		/* set the installed indicator */
	installed = INSTALL_FLAG;
	return SUCCESS;
}


/***********************************************************************
	FUNCTION : SES_Register
	CONTENTS : This will register an event routine and an associated 
		service routine.  The priority of the pairs is descending in the 
		order they were registered.
	PARAMETERS :
		aEvent -- The event detection routine.
		aService -- The associated service routine.
	RETURNS :
		unsigned char -- Flag for success (OK_OPERATION == success).  Other returns
			ERR_NOTINSTALLED
			ERR_TOOMANYEVENTS
	NOTES :
***********************************************************************/
unsigned char SES_Register(unsigned char (*aEvent)(void *), void (*aService)(void *))
{
		/* make sure all conditions are met before the change */
	if (installed != INSTALL_FLAG)
		return ERROR;
	else if (entries == MAX_ENTRIES)
		return ERROR;
	else {
		e_s[entries].event = aEvent;
		e_s[entries].service = aService;
		entries++;
		return SUCCESS;
	}
}




/***********************************************************************
	FUNCTION : SES_HandleEvents
	CONTENTS : This will run the handle events loop to process the
		events and service functions.
	PARAMETERS :
		none
	RETURNS :
		none
	NOTES :
		The scheduling is based on the scheduling set at initialization.
		This will return when a full pass through the event functions
			returns no hits.
***********************************************************************/
void SES_HandleEvents(void)
{
	unsigned char i;
	unsigned char hits;
	unsigned int data;
			
	if (!(installed == INSTALL_FLAG))
		return;

	do {
		hits = 0;
		for (i=0; i<entries; i++) {
			if (e_s[i].event(&data)) {
				e_s[i].service(&data);
				hits++;
				if (scheduleType != SES_ROUND_ROBIN)
					break;
			}
		}
	} while (hits > 0);
	
	return;
}


/***********************************************************************
	FUNCTION : SES_End
	CONTENTS : This will end the SES system.  It will basically uninstall
		the system if it was installed.
	PARAMETERS :
		none
	RETURNS :
		nothing
	NOTES :
***********************************************************************/
void SES_End(void)
{
		/* if we are installed, remove the effects */
	if (installed == INSTALL_FLAG)
		;

		/* indicate that we are uninstalled */
	installed = ~(INSTALL_FLAG);
}

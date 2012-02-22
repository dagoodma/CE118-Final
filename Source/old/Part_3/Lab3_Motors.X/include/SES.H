/***********************************************************************
	FILE : ses.h -- public header for ses.c
	CREATED : March 3, 1994
    UPDATES :
        1/7/00      Kevin Wooley    revised to handle passing of shared
                                        variables (word and byte).
		11/22/99	Matt Ohline		revised to compile with ICC11v5
***********************************************************************/




#ifndef SES_H
#define SES_H


	/* === DEFINITIONS === */
#ifndef SUCCESS
#define SUCCESS 0
#define ERROR -1
#endif
	/* --- Scheduling Algorithm --- */
#define SES_ROUND_ROBIN	1
#define SES_PRIORITY	2


	/* --- Macro defintions --- */
#define EVENT_PARAM void *key
#define SERVICE_PARAM void *key
#define SET_SHARED_BYTE_TO(z) {*((unsigned char *)key) = z;}
#define GET_SHARED_BYTE() (*((unsigned char *)key))
#define SET_SHARED_WORD_TO(z) {*((uint *)key) = z;}
#define GET_SHARED_WORD() (*((uint *) key))

	/* === FUNCTION PROTOTYPES === */
unsigned char SES_Init(unsigned char aScheduleType);
unsigned char SES_Register(unsigned char (*aEvent)(void *), void (*aService)(void *));
void SES_HandleEvents(void);
void SES_End(void);


#endif /* SES_H */


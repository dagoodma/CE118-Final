/*
 * File:   main.c
 * Author: dagoodma
 *
 * Created on February 21, 2012, 7:46 PM
 */

#define USE_MAIN

//#define DEBUG


//#define DEBUG_STATES // slows and blinks LEDs
//#define DISABLE_AVOID
//#define DEBUG_VERBOSE

//#define TAPE_CALIBRATE 1

//#define TARGET_USETIME




#ifdef USE_MAIN
#include <p32xxxx.h>
#include "serial.h"
#include "PORTS.h"
#include "timers.h"
#include "Drive.h"
#include "Gate.h"
#include "TapeSensor.h"
#include "IR_new.h"
#include "Bumper.h"
#include "AD.h"
#include "Util.h"
#include "LED.h"
//#include <stdio.h>


/*******************************************************************************
 * #DEFINES                                                                    *
 ******************************************************************************/
#define TIMER_START 6
#define TIMER_MOVE 7
#define TIMER_AVOID 8
#define TIMER_FIND 9
#define TIMER_FOLLOW 10
#define TIMER_CALIBRATE 11
#define TIMER_RETURN 12
#define TIMER_CHARGE 13
#define TIMER_OBSTACLE 14
#define MASTER_TIMER 15

#define MASTER_TIMEOUT 11000

#define START_DELAY 2500

#define CHARGE_DUMP_DELAY 800
#define CHARGE_REVERSE_DELAY 1750
#define CHARGE_TURN_DELAY 620
#define CHARGE_START_DELAY 1550

#define AVOID_FORWARD_DELAY 2750
static int avoidForwardDelay = AVOID_FORWARD_DELAY;
#define AVOID_TIMEOUT 4600
#define AVOID_REVERSE_DELAY 1300
#define AVOID_TURN_DELAY 1000
#define AVOID_BACK_DELAY 1700
#define AVOID_CHECK_DELAY 450

#define OBSTACLE_TURN_DELAY 1000
#define OBSTACLE_FORWARD_DELAY 2750
static int obstacleForwardDelay = OBSTACLE_FORWARD_DELAY;
#define OBSTACLE_REVERSE_DELAY 1300


#define LEFT_SEARCH_TIME 2750 // time before trying other direction

#define FIND_TURN_TIMEOUT 2900 // time til turn gives up
#define FIND_TURN_DELAY 790 // time to start checking right arm

#define FOLLOW_SEARCH_TIMEOUT 3000 // time to give up right turn
#define FOLLOW_ACUTE_DELAY 750
#define FOLLOW_ACUTE_TIMEOUT 5200

#define CALIBRATE_DELAY 1500
#define CALIBRATE_TIMEOUT 7600

#define CALIBRATE_INDICATOR PORTY06_LAT
#define CALIBRATE_INDICATOR_TRIS PORTY06_TRIS
#define CALIBRATE_TAPEHIGHEST_I TAPE_ARMFRONT_I
#define CALIBRATE_TAPELOWEST_I TAPE_ARMFRONT_I

#define RETURN_LEFT_DELAY 745 // time to turn left for 90 deg
#define RETURN_RIGHT_DELAY 2550 //time to turn right on left wall rebound
#define RETURN_TAPE_DELAY 2000 // time to ignore tape sensor triggers


/*******************************************************************************
 * VARIABLES                                                                   *
 ******************************************************************************/

//---------- Top state variables ------------
static enum { calibrate, target, charge, find_tape, follow_tape, return_island, hold} topState;
//static enum { none, acquired_target, lost_target, dumped, arm_on, found_island} topEvent = none;

//-------- Target state variables -----------

static unsigned int highestIRSeen = 0;
static unsigned int highestIRTime = 0;
static enum { target_findmaxleft, target_findmaxright, target_returnmax } targetState = target_findmaxleft;
static enum { target_none, target_timedout, target_highestpast, target_foundmax } targetEvent = target_none;

static unsigned char *targetStates[] = { "target_findmaxleft", "target_findmaxright", "target_returnmax" };
static unsigned char *targetEvents[] = { "target_none", "target_timedout", "target_highestpast", "target_foundmax" };

/*
static enum { target_searchleft, target_searchright, target_acquired } targetState = target_searchleft;
static enum { target_none, target_timedout, target_found, target_lost } targetEvent = target_none;
 */

//-------- Charge state variables -----------
static enum { charge_forward, charge_dump, charge_reverse, charge_turn,
        charge_avoidtape} chargeState = charge_forward;
static enum { charge_none, charge_lostbeacon, charge_hit, charge_hittape,
        charge_blocked, charge_finished, charge_avoided, charge_reversed } chargeEvent = charge_none;\
static char neverCharged = 1;

//-------- AvoidTape state variables -----------
static enum { avoid_transition, avoid_forward, avoid_left, avoid_right, avoid_back } avoidState = avoid_transition;
static enum { avoid_none, avoid_goleft, avoid_forwarded, avoid_goright, avoid_goback, avoid_cleared } avoidEvent = avoid_none;


static unsigned char *avoidStates[] = { "avoid_transition", "avoid_reverse", "avoid_turnleft", "avoid_turnright",
        "avoid_forward", "avoid_revturnleft", "avoid_bump"};
static unsigned char *avoidEvents[] = { "avoid_none", "avoid_goback", "avoid_cutright", "avoid_goright",
    "avoid_goleft", "avoid_cutleft", "avoid_reversed", "avoid_lefted", "avoid_righted",
    "avoid_forwarded", "avoid_timedout", "avoid_failed", "avoid_revover", "avoid_bumped"};

//-------- FindTape state variables -----------
static enum { find_forward, find_turn, find_avoidobstacle } findState = find_forward;
static enum { find_none, find_foundfront, find_hit, find_avoided, find_found, find_timedout } findEvent = find_none;
static unsigned char *findStates[] = { "find_forward", "find_turn" };
static unsigned char *findEvents[] = { "find_none", "find_foundfront", "find_found", "find_timedout" };

//-------- FollowTape state variables -----------
static enum { follow_transition, follow_avoidobstacle, follow_hardleft, follow_left,
        follow_hardright, follow_right, follow_forward, follow_acuteleft, follow_searchleft,
        follow_searchright } followState = follow_transition;
static enum { follow_none, follow_hit, follow_rightfrontoff, follow_rightoff,
        follow_leftfrontoff, follow_leftoff, follow_armon, follow_losttape,
        follow_foundisland, follow_foundacute, follow_acuted, follow_avoided, follow_searchfailed,
        follow_lookleft, follow_lookright, follow_foundfront, follow_acutefailed } followEvent = follow_none;

static unsigned char *followStates[] = { "follow_transition", "follow_avoidobstacle", "follow_hardleft", "follow_left",
        "follow_hardright", "follow_righ", "follow_forward", "follow_acuteleft", "follow_searchleft",
        "follow_searchright" };

static unsigned char *followEvents[] = { "follow_none", "follow_hit", "follow_rightfrontoff", "follow_rightoff",
        "follow_leftfrontoff", "follow_leftoff", "follow_armon", "follow_losttape",
        "follow_foundisland", "follow_foundacute", "follow_acuted", "follow_avoided", "follow_searchfailed",
        "follow_lookleft", "follow_lookright", "follow_foundfront", "follow_acutefailed" };

//-------- AvoidObstacle state variables -----------
static enum { obstacle_transition, obstacle_left, obstacle_right, obstacle_back, obstacle_opposite, obstacle_forward }
    obstacleState = obstacle_transition;
static enum { obstacle_none, obstacle_goleft, obstacle_goright, obstacle_goback, obstacle_cleared, obstacle_forwarded,
    obstacle_backed } obstacleEvent = obstacle_none;

static enum { return_left, return_forward, return_avoidobstacle, return_right, return_uturn } returnState = return_left;
static enum { return_none, return_lefted, return_hitobstacle, return_hitwall, return_hittape, return_uturned,
        return_righted, return_goright } returnEvent = return_none;

        static unsigned char *returnStates[] = { "return_left", "return_forward", "return_avoidobstacle", "return_right", "return_uturn" };
static unsigned char *returnEvents[] = { "return_none", "return_lefted", "return_hitobstacle", "return_hitwall", "return_armon", "return_uturned",
        "return_righted", "return_goright" };


static enum {calibrate_onthreshold, calibrate_offthreshold} calibrateState = calibrate_onthreshold;
static enum { calibrate_none, calibrate_ready, calibrate_timedout, calibrate_next, calibrate_finished } calibrateEvent = calibrate_none;

static int time;

#define START_STATE target


/*******************************************************************************
 * FUNCTION  PROTOTYPES                                                        *
 ******************************************************************************/
void HandleTopSM();

void DuringCalibrateSM();
void DuringChargeSM();
void DuringTargetSM();
void DuringAvoidTapeSM();
void DuringFindTapeSM();
void DuringFollowTapeSM();
void DuringAvoidObstacleSM();
void DuringReturnIslandSM();

void UpdateChargeEvent();
void UpdateAvoidTapeEvent();
void UpdateTargetEvent();
void UpdateFindTapeEvent();
void UpdateFollowTapeEvent();
void UpdateAvoidObstacleEvent();
void UpdateCalibrateEvent();
void UpdateReturnIslandEvent();
void UpdateCalibrateEvent();

void InitStartState();
void InitCalibrateSM();
void InitChargeSM();
void InitTargetSM();
void InitAvoidTapeSM();
void InitFindTapeSM();
void InitFollowTapeSM();
void InitReturnIslandSM();
void InitAvoidObstacleSM();
// void InitFollowAcuteSM();



// -------- others -------
void wait();
unsigned int DecrementTimer(unsigned int timeMax, unsigned int timeCur);

void DuringReturnIslandSM() {

#ifdef DEBUG
     //printf("\nReturn  STATE=%u EVENT=%u", returnState, returnEvent);
#endif
    UpdateReturnIslandEvent();

    switch(returnState) {
        case return_left:
            //printf("A");
            if (returnEvent == return_lefted) {
                //InitTimer(TIMER_RETURN, RETURN_COLLIDE_TIMEOUT);
                InitTimer(TIMER_RETURN, RETURN_TAPE_DELAY);
                returnState = return_forward;
            }
            else {
                Drive_Turn(pivot, left, HALF_SPEED);
            }
            break;
        case return_forward:
            //printf("B");
            if (returnEvent == return_hitobstacle) {
                returnState = return_avoidobstacle;
                InitAvoidObstacleSM();
            }
            else if (returnEvent == return_hittape) {
                // EXIT caller picks this up
                Drive_Stop();
            }
            /*
            else if (returnEvent == return_hitwall) {
                returnState = return_uturn;
                InitTimer(TIMER_RETURN,RETURN_LEFT_DELAY);
            }
             * */

            else if (returnEvent == return_goright) {
                returnState = return_right;
                InitTimer(TIMER_RETURN, RETURN_RIGHT_DELAY);
            }
            else {

                Drive_Forward(HALF_SPEED);
            }
            break;
        case return_avoidobstacle:
            //printf("C");
            DuringAvoidObstacleSM();
            if (avoidEvent == obstacle_forwarded) {
                returnState = return_forward;
            }
            break;
        case return_right:
            //printf("D");
            if (returnEvent == return_righted)
                returnState = return_forward;
            else
                Drive_Turn(soft,right,HALF_SPEED);
            break;
         /*
        case return_uturn:
            if (returnEvent == return_uturned)
                returnState = return_forward;
            else
                Drive_Turn(pivot, right, MID_SPEED);
            break;
       */
    } // switch
}

void UpdateReturnIslandEvent() {
    returnEvent = return_none;


    switch(returnState) {
        case return_left:
            if (IsTimerExpired(TIMER_RETURN))
                returnEvent = return_lefted;
            break;
        case return_forward:
            if (Bumper_AnyTriggered())
                returnEvent = return_hitobstacle;
            else if ((Tape_ArmLeftTriggered() || Tape_ArmRightTriggered() || Tape_ArmFrontTriggered() ||
                    Tape_RightTriggered() || Tape_CenterTriggered()) && IsTimerExpired(TIMER_RETURN))
            //else if (Tape_AnyTriggered() && IsTimerExpired(TIMER_MOVE))
                returnEvent = return_hittape;
            /**
            else if (IsTimerExpired(TIMER_RETURN) && Bumper_AnyTriggered())
                returnEvent = return_hitwall;
             **/
            else if (Tape_LeftTriggered() && IsTimerExpired(TIMER_RETURN))
                returnEvent = return_goright;
            break;
        case return_avoidobstacle:
            break;
        case return_right:
            if (IsTimerExpired(TIMER_RETURN))
                returnEvent = return_righted;
            break;
            /*
        case return_uturn:
            if (IsTimerExpired(TIMER_RETURN))
                returnEvent == return_uturned;
            break;
             * */
    } // switch
}

void DuringAvoidObstacleSM() {
    UpdateAvoidObstacleEvent();
    #ifdef DEBUG
     //printf("\nObstacle  STATE=%u EVENT=%u", obstacleState, obstacleEvent);
#endif

    switch(obstacleState) {
        case obstacle_transition:
            //printf("A");
            if (obstacleEvent == obstacle_goleft) {
                InitTimer(TIMER_OBSTACLE, OBSTACLE_TURN_DELAY);
                Drive_Turn(pivot, left, HALF_SPEED);
                obstacleState = obstacle_left;
            }
            else if (obstacleEvent == obstacle_goright) {
                InitTimer(TIMER_OBSTACLE, OBSTACLE_TURN_DELAY);
                Drive_Turn(pivot, right, HALF_SPEED);
                obstacleState = obstacle_right;
            }
            else if (obstacleEvent == obstacle_goback) {
                InitTimer(TIMER_OBSTACLE, OBSTACLE_REVERSE_DELAY);
                Drive_Reverse(HALF_SPEED);
                obstacleState = obstacle_back;
            }
            else {
                // EXIT -- caller handles
                obstacleEvent = obstacle_forwarded;
            }
            break;
        case obstacle_forward:
            //printf("B");
            if (obstacleEvent == obstacle_forwarded) {
                obstacleState = obstacle_transition;
            }
            break;
        case obstacle_back:
            //printf("C");
            if (obstacleEvent == obstacle_backed) {
                InitTimer(TIMER_OBSTACLE, OBSTACLE_TURN_DELAY);
                Drive_Turn(pivot,opposite,HALF_SPEED);
                obstacleState = obstacle_opposite;
            }
            break;
        default:
            //printf("D");
            // obstacle_opposite, left, right
            if (obstacleEvent == obstacle_cleared) {
                obstacleState = obstacle_forward;
                Drive_Forward(HALF_SPEED);
                obstacleForwardDelay = DecrementTimer(OBSTACLE_FORWARD_DELAY, obstacleForwardDelay);
                InitTimer(TIMER_OBSTACLE, obstacleForwardDelay);
            }
            break;


    /*
    switch(obstacleState) {
        case obstacle_reverse:
            if (obstacleEvent == obstacle_reversed) {
                InitTimer(TIMER_MOVE, OBSTACLE_TURN_DELAY);
                obstacleState = obstacle_turnleft;
            }
            else {
                Drive_Reverse(HALF_SPEED);
            }
            break;
        case obstacle_turnleft:
            if (obstacleEvent == obstacle_lefted) {
                InitTimer(TIMER_MOVE,OBSTACLE_FORWARD_DELAY);
                obstacleState = obstacle_forward;
            }
            else {
                Drive_Turn(pivot, left, HALF_SPEED);
            }
            break;
        case obstacle_forward:
            if (obstacleEvent == obstacle_forwarded) {
                InitTimer(TIMER_MOVE,OBSTACLE_TURN_DELAY);
                obstacleState = obstacle_turnright;
            }
            else {
                Drive_Forward(HALF_SPEED);
            }
            break;
        case obstacle_turnright:
            if (obstacleEvent == obstacle_righted) {
                InitTimer(TIMER_MOVE,OBSTACLE_FORWARD2_DELAY);
                obstacleState = obstacle_forward2;
            }
            else {
                Drive_Turn(pivot, right, HALF_SPEED);
            }
            break;
        case obstacle_forward2:
            if (obstacleEvent == obstacle_forwarded) {
                InitTimer(TIMER_MOVE,OBSTACLE_TURN2_DELAY);
                obstacleState = obstacle_turnright2;
            }
            else {
                Drive_Forward(HALF_SPEED);
            }
            break;
        case obstacle_turnright2:
            if (obstacleEvent == obstacle_avoided) {
                // EXIT -- caller handles this
                Drive_Stop();
            }
            break;
     */

    } // switch
}

void UpdateAvoidObstacleEvent() {
    obstacleEvent = obstacle_none;
    switch(obstacleState) {
        case obstacle_transition:
            if (Bumper_RightTriggered() && ! Bumper_LeftTriggered())
                obstacleEvent = obstacle_goleft;
            else if (!Bumper_RightTriggered() && Bumper_LeftTriggered())
                obstacleEvent = obstacle_goright;
            else if (Bumper_LeftTriggered() && Bumper_RightTriggered()
                    || Bumper_CenterTriggered())
                obstacleEvent = obstacle_goback;
            else
                obstacleEvent = obstacle_cleared;
            break;
        case obstacle_left:
            if (!Bumper_RightTriggered() && IsTimerExpired(TIMER_OBSTACLE))
                obstacleEvent = obstacle_cleared;
            break;
        case obstacle_right:
            if (!Bumper_LeftTriggered() && IsTimerExpired(TIMER_OBSTACLE))
                obstacleEvent = obstacle_cleared;
            break;
        case obstacle_back:
            if (!Bumper_CenterTriggered() && IsTimerExpired(TIMER_OBSTACLE)
                    || Tape_BackTriggered())
                obstacleEvent = obstacle_backed;
            break;
        case obstacle_opposite:
            if (IsTimerExpired(TIMER_OBSTACLE))
                obstacleEvent = obstacle_cleared;
            break;
        case obstacle_forward:
            if (IsTimerExpired(TIMER_OBSTACLE) || Tape_LeftTriggered()
                    || Tape_CenterTriggered() || Tape_RightTriggered())
                obstacleEvent = obstacle_forwarded;
            break;


    } // switch

    /*
    switch(obstacleState) {
        case obstacle_reverse:
            if (IsTimerExpired(TIMER_MOVE) || Tape_BackTriggered())
                obstacleEvent = obstacle_reversed;
            break;
        case obstacle_turnleft:
            if (IsTimerExpired(TIMER_MOVE)) {
                avoidEvent = obstacle_lefted;
            }
            break;
        case obstacle_forward:
            if (IsTimerExpired(TIMER_MOVE)) {
                avoidEvent = obstacle_forwarded;
            }
            break;
        case obstacle_turnright:
            if (IsTimerExpired(TIMER_MOVE)) {
                avoidEvent = obstacle_righted;
            }
            break;
        case obstacle_forward2:
            if (IsTimerExpired(TIMER_MOVE)) {
                avoidEvent = obstacle_forwarded;
            }
            break;
        case obstacle_turnright2:
            if (IsTimerExpired(TIMER_MOVE)) {
                avoidEvent = obstacle_avoided;
            }
            break;
     */

}


/*
 * @Function DuringFollowTapeSM()
 * @remark
 * @date 2012.3.6 08:30 */
void DuringFollowTapeSM() {
    UpdateFollowTapeEvent();

  /*  if(time<(GetTime()-100)){
     
     time = GetTime();
    }
    */
#ifdef DEBUG
    //printf("\nFollow tape STATE=%u EVENT=%u", followState, followEvent);
#endif


    switch (followState) {
        case follow_transition:
            //printf("A");
            if (followEvent == follow_hit) {
                Drive_Stop();
                InitAvoidObstacleSM();
                followState = follow_avoidobstacle;
            }
            else if (followEvent == follow_foundisland) {
                Drive_Stop();
                // EXIT - caller handles
            }
            else if (followEvent == follow_foundacute) {
                Drive_Stop();
                InitTimer(TIMER_MOVE, FOLLOW_ACUTE_DELAY);
                InitTimer(TIMER_FOLLOW, FOLLOW_ACUTE_TIMEOUT);
                followState = follow_acuteleft;
            }
            else if (followEvent == follow_lookleft) {
                followState = follow_searchleft;
            }
            else if (followEvent == follow_lookright) {
                followState = follow_searchright;
                InitTimer(TIMER_MOVE, FOLLOW_SEARCH_TIMEOUT);
            }
            else if (followEvent == follow_rightfrontoff) {
                followState = follow_hardleft;
            }
            else if (followEvent == follow_rightoff) {
                followState = follow_left;
            }
            else if (followEvent == follow_leftfrontoff) {
                followState = follow_hardright;
            }
            else if (followEvent == follow_leftoff) {
                followState = follow_right;
            }
            else if (followEvent == follow_armon) {
                followState = follow_forward;
            }
            else {
                Drive_Stop();
                // losttape!
                // EXIT - caller handles
            }
            break;
        case follow_avoidobstacle:
            //printf("B");
            DuringAvoidObstacleSM();
            if (followEvent == follow_avoided)
                followState = follow_transition;
            break;
        case follow_searchright:
            //printf("C");
            if (followEvent == follow_searchfailed) {
                InitTimer(TIMER_MOVE, FOLLOW_SEARCH_TIMEOUT);
                followState = follow_searchleft;
            }
            else if (followEvent == follow_foundfront) {
                followState = follow_transition;
                Drive_Stop();
            }
            else {
                Drive_Turn(pivot, right, MIN_SPEED);
            }
            break;
        case follow_searchleft:
            //printf("D");
            if (followEvent == follow_searchfailed) {
                followState = follow_transition;
            }
            else if (followEvent == follow_foundfront)
                followState = follow_transition;
            else
                Drive_Turn(pivot, left,  MIN_SPEED);
            break;
        case follow_acuteleft:
            //printf("E");
            if (followEvent == follow_acuted) {

                followState = follow_transition;
            }
            else if(followEvent == follow_acutefailed) {
                followState = follow_transition;
            }
            else {
                Drive_Turn(pivot, left, FULL_SPEED);
            }
            break;
        case follow_hardleft:
            //printf("F");
            Drive_Turn(hard,left,MID_SPEED);
            followState = follow_transition;
            break;
        case follow_left:
            //printf("G");
            Drive_Turn(hard,left,HALF_SPEED);
            followState = follow_transition;
            break;
        case follow_hardright:
            //printf("H");
            Drive_Turn(hard,right,MID_SPEED);
            followState = follow_transition;
            break;
        case follow_right:
            //printf("I");
            Drive_Turn(hard,right,HALF_SPEED);
            followState = follow_transition;
            break;
        case follow_forward:
            //printf("J");
            Drive_Forward(HALF_SPEED);
            followState = follow_transition;
            break;

    } // switch
}

/*
 * @Function UpdateFollowTapeEvent()
 * @remark
 * @date 2012.3.6 08:30 */
void UpdateFollowTapeEvent() {
    followEvent = follow_none;

    switch (followState) {
        case follow_transition:
            if (Bumper_AnyTriggered() ) //|| ( IR_MainTriggered() && IR_AngleTriggered()))
                followEvent = follow_hit;
            else if ((Tape_LeftTriggered() && ! Tape_RightTriggered() && IsTimerExpired(TIMER_FOLLOW)) ||
                    (Tape_LeftTriggered() && ! Tape_CenterTriggered() && IsTimerExpired(TIMER_FOLLOW)))
                followEvent = follow_foundacute;
            else if (Tape_LeftTriggered() && Tape_CenterTriggered() && Tape_RightTriggered())
                followEvent = follow_foundisland;
            
            else if ( Tape_ArmLeftTriggered() && Tape_ArmFrontTriggered() && Tape_ArmRightTriggered()
                    || !Tape_ArmLeftTriggered() && Tape_ArmFrontTriggered() && !Tape_ArmRightTriggered())
                followEvent = follow_armon;
            else if ( ! Tape_ArmLeftTriggered() && ! Tape_ArmFrontTriggered() && ! Tape_ArmRightTriggered() &&
                    ! Tape_RightTriggered())
                followEvent = follow_losttape;
            else if ( ! Tape_ArmFrontTriggered() &&  Tape_ArmLeftTriggered() && Tape_ArmRightTriggered()
                    && Tape_RightTriggered() )
                followEvent = follow_lookleft;
            else if ( ! Tape_ArmFrontTriggered() &&  Tape_ArmLeftTriggered() && Tape_ArmRightTriggered())
                followEvent = follow_lookright;
            else if ((Tape_ArmLeftTriggered() || Tape_RightTriggered()) && ! Tape_ArmFrontTriggered() && ! Tape_ArmRightTriggered())
                followEvent = follow_rightfrontoff;
            else if (Tape_ArmLeftTriggered() && Tape_ArmFrontTriggered() && ! Tape_ArmRightTriggered())
                followEvent = follow_rightoff;
            else if ( ! Tape_ArmLeftTriggered() && ! Tape_ArmFrontTriggered() && Tape_ArmRightTriggered())
                followEvent = follow_leftfrontoff;
            else if ( ! Tape_ArmLeftTriggered() && Tape_ArmFrontTriggered() && Tape_ArmRightTriggered())
                followEvent = follow_leftoff;
            break;
        case follow_avoidobstacle:
            if (obstacleEvent == obstacle_forwarded)
                followEvent = follow_avoided;
            break;
        case follow_searchright:
            if (Tape_ArmFrontTriggered())
                followEvent = follow_foundfront;
            else if (IsTimerExpired(TIMER_MOVE))
                followEvent = follow_searchfailed;
            break;
        case follow_searchleft:
            if (Tape_ArmFrontTriggered())
                followEvent = follow_foundfront;
            else if (IsTimerExpired(TIMER_MOVE))
                followEvent = follow_searchfailed;
            break;
        case follow_acuteleft:
            if (IsTimerExpired(TIMER_MOVE)
                    && (Tape_ArmLeftTriggered() || Tape_ArmFrontTriggered() || Tape_ArmRightTriggered() ))
                followEvent = follow_acuted;
            else if (IsTimerExpired(TIMER_FOLLOW))
                followEvent = follow_acutefailed;
            break;
    } // switch
}


/*
 * @Function DuringFollowTapeSM()
 * @remark
 * @date 2012.3.6 08:30 */
void DuringFindTapeSM() {
    UpdateFindTapeEvent();
#ifdef DEBUG
    //printf("\nFind tape STATE=%u EVENT=%u", findState, findEvent);
#endif

    switch (findState) {
        case find_forward:
            //printf("A");
            if (findEvent == find_foundfront) {
                InitTimer(TIMER_FIND, FIND_TURN_TIMEOUT);
                InitTimer(TIMER_MOVE, FIND_TURN_DELAY);
                findState = find_turn;
            }
            else if (findEvent == find_hit) {
                Drive_Stop();
                InitAvoidObstacleSM();
                findState = find_avoidobstacle;
            }
            else {
                Drive_Forward(HALF_SPEED);
            }
            break;
        case find_avoidobstacle:
        //printf("B");
            DuringAvoidObstacleSM();
            if (findEvent == find_avoided)
                findState = find_forward;;
            break;
        case find_turn:
//printf("C");
            if (findEvent == find_found) {
                // EXIT -- caller picks up
                Drive_Stop();
            }
            else if (findEvent == find_timedout) {
                findState = find_forward;
                Drive_Forward(FULL_SPEED);
                wait();
                Drive_Stop();
                ClearTimerExpired(TIMER_FIND);
            }
            else {
                Drive_Turn(pivot, left, HALF_SPEED);
            }
            break;

    } // switch
}

/*
 * @Function UpdateFindTapeEvent()
 * @remark
 * @date 2012.3.6 08:30 */
void UpdateFindTapeEvent() {
    findEvent = find_none;

    switch (findState) {
        case find_forward:
            if (Tape_ArmRightTriggered() || Tape_ArmFrontTriggered() ||
                    Tape_ArmLeftTriggered() )
                findEvent = find_found;

            else if (Bumper_AnyTriggered())
                findEvent = find_hit;
            else if (Tape_LeftTriggered() || Tape_CenterTriggered()
                    || Tape_RightTriggered())
                findEvent = find_foundfront;
            break;
        case find_avoidobstacle:
            if (obstacleEvent == obstacle_forwarded)
                findEvent = find_avoided;
            break;
        case find_turn:
            if ( IsTimerExpired(TIMER_MOVE) && (Tape_ArmRightTriggered()
                    || Tape_ArmFrontTriggered() || Tape_ArmLeftTriggered()) )
                findEvent = find_found;
            else if (IsTimerExpired(TIMER_FIND))
                findEvent = find_timedout;
            break;
    } // switch
}

/*
 * @Function DuringTargetSM
 * @remark
 * @date 2012.3.6 08:30 */
void DuringTargetSM() {
    UpdateTargetEvent();
////printf("\nHandling TargetSM STATE=%s, EVENT=%s", targetStates[targetState], targetEvents[targetEvent]);
#ifdef DEBUG
     //printf("\nHandling TargetSM STATE=%u, EVENT=%u", targetState, targetEvent);
#endif
    /*
    switch (targetState) {
        case target_searchleft:
            if (targetEvent == target_found)
                targetState = target_acquired;
            else if (targetEvent == target_timedout)
                targetState = target_searchright;
            else
                Drive_Turn(pivot,left, HALF_SPEED);
            break;
        case target_searchright:
            if (targetEvent == target_found)
                targetState = target_acquired;
            else
                Drive_Turn(pivot,right, HALF_SPEED);
            break;
        case target_acquired:
            if (targetEvent == target_lost)
                InitTargetSM();
            else
                Drive_Stop();
                // EXIT -- caller picks up
            break;
    } // switch
     */

    switch (targetState) {
        case target_findmaxleft:
            //printf("A");
            if (targetEvent == target_highestpast) {
                targetState = target_returnmax;
                Drive_Turn(pivot, right, MID_SPEED);
                #ifdef TARGET_USETIME
                int returnTime = (GetTime() - highestIRTime);
                InitTimer(TIMER_MOVE, returnTime);
                #endif
            }
            else if (targetEvent == target_timedout) {
                targetState = target_findmaxright;
            }
            else {
                Drive_Turn(pivot,left,MID_SPEED);
            }
            break;
        case target_findmaxright:
            //printf("B");
            if (targetEvent == target_highestpast) {
                targetState = target_returnmax;
                Drive_Turn(pivot, left, MIN_SPEED);
#ifdef TARGET_USETIME
                int returnTime = (GetTime() - highestIRTime);
                InitTimer(TIMER_MOVE, returnTime);
#endif
            }
            else {
                Drive_Turn(pivot,right,HALF_SPEED);
            }
            break;
        case target_returnmax:
            //printf("C");
            if (targetEvent == target_foundmax) {
                Drive_Stop();
                //wait();
                //wait();
                // EXIT -- caller picks this up
            }
            break;
        default:
            break;

#ifdef DEBUG
             //db//printf("\nHorrible error occured!");
#endif

    } // switch

} // DuringTargetSM()


/*
 * @Function UpdateTargetEvent
 * @remark
 * @date 2012.3.6 08:30 */
void UpdateTargetEvent() {
    unsigned int reading = IR_MainReading();
    targetEvent = target_none;
/*
    switch (targetState) {
        case target_searchleft:
            if (IR_MainTriggered())
                targetEvent = target_found;
            else if (IsTimerExpired(TIMER_MOVE))
                targetEvent = target_timedout;
            break;
        case target_searchright:
            if (IR_MainTriggered())
                targetEvent = target_found;
            break;
        case target_acquired:
            if (!IR_MainTriggered())
                targetEvent = target_lost;
            break;
    } // switch
 */
     // TODO use timer for returnmax

    switch (targetState) {
        case target_findmaxleft:
            if (reading >= highestIRSeen && IR_MainTriggered()) {
                highestIRSeen = reading;
                highestIRTime = GetTime();
            }
            else if (IsTimerExpired(TIMER_MOVE)) {
                targetEvent = target_timedout;
            }
            else if (reading <(highestIRSeen -100) && highestIRSeen >150) {

#ifdef DEBUG
                 //db////printf("\nReading %u, highest %u", reading, highestIRSeen);
#endif
                // found max and dropping
                targetEvent = target_highestpast;
            }
            break;
        case target_findmaxright:
            if (reading >= highestIRSeen && IR_MainTriggered()) {
                highestIRSeen = reading;
                highestIRTime = GetTime();
            }
            else if (reading <(highestIRSeen - 100) && highestIRSeen>150) {
                // found max and dropping
                targetEvent = target_highestpast;
            }
            break;
        case target_returnmax:
            //
#ifdef TARGET_USETIME
            if (IsTimerExpired(TIMER_MOVE))
#else
            if (reading >= (highestIRSeen - 20))
#endif
                targetEvent = target_foundmax;
            break;
    } // switch(

} // UpateTargetEvent()

/*
 * @Function DuringChargeSM
 * @remark
 * @date 2012.3.6 08:30 */
void DuringChargeSM() {
    UpdateChargeEvent();
    #ifdef DEBUG
    ////printf("\nHandling ChargeSM %u %u", chargeEvent, chargeState);
#endif

#ifdef DEBUG
     //db////printf("\nHandling ChargeSM %u", chargeEvent);
#endif

    switch (chargeState) {
        case charge_forward:
            ////printf("A");
            if (chargeEvent == charge_lostbeacon) {
                // EXIT - caller picks this up
            }
            else if(chargeEvent == charge_hit) {
                chargeState = charge_dump;
                Drive_Stop();
                Gate_Open();
                InitTimer(TIMER_MOVE, CHARGE_DUMP_DELAY);
            }
            else if(chargeEvent == charge_hittape) {
                Drive_Stop();
                InitAvoidTapeSM();
                chargeState = charge_avoidtape;
            }
            else {
                Drive_Forward(FULL_SPEED);
            }
            break;
        case charge_avoidtape:
            //printf("B");
            DuringAvoidTapeSM();
            if (chargeEvent == charge_avoided)
                chargeState = charge_forward;

            break;
        case charge_dump:
            //printf("C");
            if (chargeEvent == charge_blocked) {
                chargeState = charge_reverse;
                InitTimer(TIMER_MOVE, CHARGE_REVERSE_DELAY);
            }
            else if (chargeEvent == charge_finished) {
                Drive_Stop();
                // EXIT - caller picks this up
            }
            break;
        case charge_reverse:
            //printf("D");
            if (chargeEvent == charge_reversed) {
                chargeState = charge_turn;
                Drive_Stop();
                InitTimer(TIMER_MOVE, CHARGE_TURN_DELAY);
            }
            else {
                Drive_Reverse(HALF_SPEED);
            }
            break;
        case charge_turn:
            //printf("E");
            if (chargeEvent == charge_finished) {
                // EXIT - caller picks this up
                Drive_Stop();
            }
            else {
                Drive_Turn(pivot, right, FULL_SPEED);
            }
            break;

        default:
            break;

#ifdef DEBUG
            //db//printf("\nA horrible error occured!");
#endif
    } // switch
}

/*
 * @Function UpdateChargeEvent
 * @remark
 * @date 2012.3.6 08:30 */
void UpdateChargeEvent() {
    chargeEvent = charge_none;

    switch (chargeState) {
        case charge_forward:
            // TODO improve define names
            if(!IR_MainTriggered()) {
            //if (!1) {
                chargeEvent = charge_lostbeacon;
            }
            else if(Tape_AnyTriggered() && IsTimerExpired(TIMER_CHARGE)
                && !(IR_MainTriggered() && IR_AngleTriggered()) ) {
                // hit tape, start timer expired,
                // and enemy is not right in front
                chargeEvent = charge_hittape;
            }
            else if(Bumper_AnyTriggered() || (IR_AngleTriggered() && IR_MainTriggered())) {
                chargeEvent = charge_hit;
            }
            break;
        case charge_avoidtape:
            if ( avoidEvent == avoid_forwarded )
                chargeEvent = charge_avoided;
            break;
        case charge_dump:
            if (IsTimerExpired(TIMER_MOVE)) {
                if (IR_MainTriggered() && IR_AngleTriggered())
                    chargeEvent = charge_blocked;
                else
                    chargeEvent = charge_finished;
            }
            break;
        case charge_reverse:
            if (IsTimerExpired(TIMER_MOVE) || Tape_BackTriggered())
                chargeEvent = charge_reversed;
            break;

        case charge_turn:
            if (IsTimerExpired(TIMER_MOVE) || Tape_LeftTriggered())
                chargeEvent = charge_finished;
            break;


    } // switch
}
/*
  * @Function DuringAvoidTapeSM
 * @remark
 * @date  */
void DuringAvoidTapeSM() {
    UpdateAvoidTapeEvent();

#ifdef DEBUG
     //printf("\nHandling AvoidSM STATE=%u EVENT=%u", avoidState, avoidEvent);
#endif
    switch (avoidState) {
        case avoid_transition:
            //printf("A");
            if (avoidEvent == avoid_goright) {
                InitTimer(TIMER_AVOID, AVOID_TURN_DELAY);
                avoidState = avoid_right;
                Drive_Turn(pivot, right, HALF_SPEED);
            }
            else if (avoidEvent == avoid_goleft) {
                InitTimer(TIMER_AVOID, AVOID_TURN_DELAY);
                avoidState = avoid_left;
                Drive_Turn(pivot, left, HALF_SPEED);
            }
            else if (avoidEvent == avoid_goback) {
                InitTimer(TIMER_AVOID, AVOID_BACK_DELAY);
                avoidState = avoid_back;
                Drive_Reverse(HALF_SPEED);
            }
            else { //if (avoidEvent == avoid_cleared) {
                // EXIT -- caller handles
                Drive_Stop();
            }
            break;
        case avoid_forward:
            //printf("B");
            if (avoidEvent == avoid_forwarded)
                avoidState = avoid_transition;
            break;
        default:
            //printf("C");
            if (avoidEvent == avoid_cleared) {
                avoidState = avoid_forward;
                Drive_Forward(HALF_SPEED);
                avoidForwardDelay = DecrementTimer(AVOID_FORWARD_DELAY, avoidForwardDelay);
                InitTimer(TIMER_AVOID, avoidForwardDelay);
            }
            break;
        

    }//switch
}

/*
 * @Function UpdateAvoidTapeEvent
 * @remark
 * @date 2012.3.6 08:30 */
void UpdateAvoidTapeEvent() {
    avoidEvent = avoid_none;

    switch(avoidState){
        case avoid_transition:
            if (Tape_LeftTriggered())
                avoidEvent = avoid_goright;
            else if (Tape_AnyRightTriggered())
                avoidEvent = avoid_goleft;
            else if (Tape_CenterTriggered())
                avoidEvent = avoid_goback;
            else
                avoidEvent = avoid_forwarded;
            break;
        case avoid_forward:
            if (IsTimerExpired(TIMER_AVOID) || Tape_LeftTriggered()
                    || Tape_CenterTriggered() || Tape_RightTriggered())
                avoidEvent = avoid_forwarded;
            break;
        case avoid_right:
            if (!Tape_LeftTriggered() && IsTimerExpired(TIMER_AVOID))
                avoidEvent = avoid_cleared;
            break;
        case avoid_left:
            if (!Tape_AnyRightTriggered() && IsTimerExpired(TIMER_AVOID))
                avoidEvent = avoid_cleared;
            break;
        case avoid_back:
            if (!Tape_CenterTriggered() && IsTimerExpired(TIMER_AVOID))
                avoidEvent = avoid_cleared;
            break;
           
    }//switch

}

/*
  * @Function DuringCalibrateSM()
 * @remark
 * @date  */
void DuringCalibrateSM() {
    UpdateCalibrateEvent();

#ifdef DEBUG
     ////printf("\nHandling CalibrateSM STATE=%u EVENT=%u", calibrateState, calibrateEvent);
#endif
    switch (calibrateState) {
        case calibrate_onthreshold:
            if (calibrateEvent == calibrate_next) {
                Tape_SetOnTapeThreshold(CALIBRATE_TAPEHIGHEST_I);

#ifdef DEBUG
                 ////printf("\nReady for off tape calibration. Trigger front bumper when done.");
#endif
                InitTimer(TIMER_MOVE,CALIBRATE_DELAY);
                CALIBRATE_INDICATOR = 0;
                calibrateState = calibrate_offthreshold;
            }
            else if (calibrateEvent == calibrate_timedout) {
                // EXIT -- caller picks this up
                CALIBRATE_INDICATOR = 0;
            }
            else {
                CALIBRATE_INDICATOR = 1;
            }
            break;
        case calibrate_offthreshold:
            if (calibrateEvent == calibrate_finished) {
                ClearTimerExpired(TIMER_MOVE);
                Tape_SetOffTapeThreshold(CALIBRATE_TAPELOWEST_I);
                CALIBRATE_INDICATOR = 0;
            }
            else if (calibrateEvent == calibrate_ready) {
                CALIBRATE_INDICATOR = 1;
            }
            break;

    }//switch
}

/*
 * @Function UpdateCalibrateEvent
 * @remark
 * @date 2012.3.6 08:30 */
void UpdateCalibrateEvent() {
    calibrateEvent = calibrate_none;

    switch (calibrateState) {
        case calibrate_onthreshold:
            if (Bumper_CenterTriggered()) {
                calibrateEvent = calibrate_next;
            }
            else if (IsTimerExpired(TIMER_CALIBRATE)) {
                //calibrateEvent = calibrate_timedout;
            }
            break;
        case calibrate_offthreshold:
            if (Bumper_CenterTriggered() && IsTimerExpired(TIMER_MOVE))
                calibrateEvent = calibrate_finished;
            else if (IsTimerExpired(TIMER_MOVE)) {
                calibrateEvent = calibrate_ready;
            }
            break;

    }//switch

}

// ----------------------- Main state machine handler -----------------
/*
 * @Function HandleTopSM
 * @remark
 * @date 2012.3.6 08:30 */
void HandleTopSM() {
    #ifdef DEBUG
    //printf("\nTopstate: %u", topState);
    #endif

    // Sub state machines handle topEvents
    switch (topState) {
        case calibrate:
            DuringCalibrateSM();


             ////printf("\nHere with event=%d",calibrateEvent);

            if (calibrateEvent == calibrate_timedout
                    || calibrateEvent == calibrate_finished) {
                InitStartState();
                wait();
                InitTimer(TIMER_START, START_DELAY);
            }// if
            break;

        case target:
            //printf("a");
            DuringTargetSM();
            if (targetEvent == target_foundmax) {
                //topState = hold;
                topState = charge;
                InitChargeSM();
            }

            break;

        case charge:
            //printf("b");
            DuringChargeSM();
            if (chargeEvent == charge_finished)  {
                // TODO code rest of state machine
                Drive_Stop();
                topState = find_tape;
                InitFindTapeSM();
            }
            else if (chargeEvent == charge_lostbeacon) {
                InitTargetSM();
                topState = target;
            }
            break;
        case find_tape:
            //printf("c");
            DuringFindTapeSM();
            if (findEvent == find_found) {
                InitFollowTapeSM();
                topState = follow_tape;
            }
            else if (IsTimerExpired(MASTER_TIMER)) {
                Drive_Forward(FULL_SPEED);
                wait();
                Drive_Stop();
                InitFindTapeSM();
            }
            break;
        case follow_tape:
            //printf("d");
            DuringFollowTapeSM();
            if (followEvent == follow_losttape) {
                InitFindTapeSM();
                topState = find_tape;
                //topState = hold;
            }
            else if (followEvent == follow_foundisland) {

#ifdef DEBUG
                 ////printf("\nHERR!");
#endif
                topState = return_island;
                InitReturnIslandSM();
                //topState = return_island;
            }
            break;
            // TODO code rest of state machine

        case return_island:
            //printf("e");
            DuringReturnIslandSM();
            if (returnEvent == return_hittape){
                InitFindTapeSM();
                topState = find_tape;
            }
            // TODO code rest of state machine
            break;
        default:
            break;

#ifdef DEBUG
           //db//printf("\nHorrible error occured!");
#endif
    } // switch
 }

/*******************************************************************************
 * ENTRY POINT                                                                 *
 ******************************************************************************/

int main(void) {

    // ---------------------------- Initialization ----------------------------
    SERIAL_Init();
    TIMERS_Init();

    // Initialize interrupts
    INTEnableSystemMultiVectoredInt();
      time = GetTime();

     // Initialize AD system
    /*
    if ( AD_Init(TAPE_LEFT | TAPE_CENTER | TAPE_RIGHT | TAPE_BACK |
        TAPE_ARMFRONT | TAPE_ARMLEFT | TAPE_ARMRIGHT | IR_PINS | BAT_VOLTAGE )
            == SUCCESS) {

#ifdef DEBUG
         //db//printf("\nADC initialized successfully.");
#endif
    }
    else {

#ifdef DEBUG
         //db//printf("\nADC failed to initialize.");
#endif
    }
     */
    int adPins = TAPE_LEFT | TAPE_CENTER | TAPE_RIGHT | TAPE_BACK |
         TAPE_ARMFRONT | TAPE_ARMLEFT | TAPE_ARMRIGHT | BAT_VOLTAGE | IR_PINS;

    AD_Init(adPins);

#ifdef DEBUG
     ////printf("\nPins=%x", adPins);
#endif

    // Initialize modules
    IR_Init();
    Tape_Init();
    Drive_Init();
    Bumper_Init();
    Gate_Init();

     time = GetTime();

    // Initialize state machine


#ifdef TAPE_CALIBRATE
    InitCalibrateSM();
        
    topState = calibrate;
#else

    
#endif

    InitTimer(TIMER_START, START_DELAY);

     while (!IsTimerExpired(TIMER_START)) {
        Tape_HandleSM();
        Drive_Update();
        Bumper_Update();
        IR_Update();

        if (Bumper_AnyTriggered()) {
            /*
            Drive_Turn(pivot,left,HALF_SPEED);
            wait();
            Drive_Turn(pivot,right,HALF_SPEED);
            wait();
            Drive_Turn(pivot,left,HALF_SPEED);
            wait();
            Drive_Turn(pivot,right,HALF_SPEED);
            wait();
            Drive_Reverse(FULL_SPEED);
            wait();
            Drive_Forward(FULL_SPEED);
             */
            wait();

            topState = hold;
            
        }

        if (topState == hold) {
                goto exit;
        }
     }

    InitStartState();

#ifdef DEBUG
     //printf("\nReady, set...");
    wait();
     //printf(" Go!");

     //db//printf("\nHello, I am working...");
#endif





    // ------------------------------- Main Loop -------------------------------
    while (1) {
        // Handle updates and module state machines
        Tape_HandleSM();
        Drive_Update();
        Bumper_Update();
        IR_Update();
        
        HandleTopSM();


        


#ifdef DEBUG
         ////printf("\nTopSTATE! %u", topState);
#endif

        if (topState == hold) {
                goto exit;
            }

       //while (!IsTransmitEmpty()); // bad, this is blocking code

    }
    exit:
    Tape_End();
    Drive_Stop();
    Bumper_End();
    Gate_End();
    IR_End();

    return 0;
} // end of main()

void InitStartState() {
    topState = START_STATE;
    switch (START_STATE) {
        case charge:
            InitChargeSM();
            break;
        case find_tape:
            InitFindTapeSM();
            break;
        case follow_tape:
            InitFollowTapeSM();
            break;
        case target:
            InitTargetSM();
            break;

    } // switch
}


void InitChargeSM() {
    if (neverCharged) {
        InitTimer(TIMER_CHARGE, CHARGE_START_DELAY);
        neverCharged = 0;
    }
    ClearTimerExpired(TIMER_MOVE);
    chargeState = charge_forward;
}

void InitFollowTapeSM() {
    followState = follow_transition;
    InitTimer(TIMER_FOLLOW,1);
}

void InitCalibrateSM() {
    calibrateState = calibrate_onthreshold;

#ifdef DEBUG
//     //printf("\nReady for on tape threshold calibration. Trigger front bumper when done.");
#endif
    InitTimer(TIMER_CALIBRATE, CALIBRATE_TIMEOUT);
    CALIBRATE_INDICATOR_TRIS = 0;
    CALIBRATE_INDICATOR = 0;
}

void InitFindTapeSM() {
    findState = find_forward;
    InitTimer(MASTER_TIMER,MASTER_TIMEOUT);
}

void InitAvoidObstacleSM() {
    obstacleState = obstacle_transition;
}

void InitReturnIslandSM() {
    returnState = return_left;
    InitTimer(TIMER_RETURN, RETURN_LEFT_DELAY);
}

void InitTargetSM() {

    targetState = target_findmaxleft;
    highestIRSeen = 0;
    highestIRTime = 0;
    /*
    targetState = target_searchleft;
    InitTimer(TIMER_MOVE, TARGET_FIRST_SEARCH_TIME);
     */
    InitTimer(TIMER_MOVE, LEFT_SEARCH_TIME);
}

void InitAvoidTapeSM() {
    //tryedAvoiding = 0;
    avoidState = avoid_transition;
    //InitTimer(TIMER_AVOID, AVOID_TIMEOUT);
}

/**
* Function: wait
* @return Nothing
* @remark Waits a small period of time
*/
void wait() {
    unsigned int wait = 0;
    for (wait = 0; wait <= 1000000; wait++)
        asm("nop");
}

unsigned int DecrementTimer(unsigned int timeMax, unsigned int timeCur) {
    int time = timeCur - (timeMax >> 2);
    if (time < 0)
        return timeMax;
    else
        return (unsigned int)time;
}

// ---------------------- EOF ----------------------
#endif

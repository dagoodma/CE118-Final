/*
 * File:   main.c
 * Author: dagoodma
 *
 * Created on February 21, 2012, 7:46 PM
 */

#define USE_MAIN

//#define DEBUG_STATES // slows and blinks LEDs
//#define DISABLE_AVOID
//#define DEBUG_VERBOSE

#define TAPE_CALIBRATE 1

#ifdef DEBUG_VERBOSE
#define dbprintf(...) dbprintf(__VA_ARGS__)
#else
#define dbprintf(...)
#endif



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

#define START_DELAY 2500

#define CHARGE_DUMP_DELAY 2000
#define CHARGE_REVERSE_DELAY 2000
#define CHARGE_TURN_DELAY 2500

#define AVOID_FORWARD_DELAY 1200
#define AVOID_TIMEOUT 7200
#define AVOID_REVERSE_DELAY 2100
#define AVOID_TURN_DELAY 1900

#define OBSTACLE_TURN_DELAY 1750
#define OBSTACLE_FORWARD_DELAY 3500
#define OBSTACLE_REVERSE_DELAY 1350

#define LEFT_SEARCH_TIME 2450 // time before trying other direction

#define FIND_TURN_TIMEOUT 4000 // time til turn gives up
#define FIND_TURN_DELAY 2500 // time to start checking right arm

#define FOLLOW_SEARCH_TIMEOUT 3000 // time to give up right turn
#define FOLLOW_ACUTE_DELAY 2150
#define FOLLOW_ACUTE_TIMEOUT 3700

#define CALIBRATE_DELAY 1500
#define CALIBRATE_TIMEOUT 7600

#define CALIBRATE_INDICATOR PORTY06_LAT
#define CALIBRATE_INDICATOR_TRIS PORTY06_TRIS
#define CALIBRATE_TAPEHIGHEST_I TAPE_ARMFRONT_I
#define CALIBRATE_TAPELOWEST_I TAPE_RIGHT_I

#define RETURN_COLLIDE_TIMEOUT 5000
#define RETURN_LEFT_DELAY 2550
#define RETURN_RIGHT_DELAY 2550


/*******************************************************************************
 * VARIABLES                                                                   *
 ******************************************************************************/

//---------- Top state variables ------------
static enum { calibrate, target, charge, find_tape, follow_tape, return_island, hold} topState;
//static enum { none, acquired_target, lost_target, dumped, arm_on, found_island} topEvent = none;

//-------- Target state variables -----------

static unsigned int highestIRSeen = 0;
static enum { target_findmaxleft, target_findmaxright, target_returnmax } targetState = target_findmaxleft;
static enum { target_none, target_timedout, target_highestpast, target_foundmax } targetEvent = target_none;

/*
static enum { target_searchleft, target_searchright, target_acquired } targetState = target_searchleft;
static enum { target_none, target_timedout, target_found, target_lost } targetEvent = target_none;
 */

//-------- Charge state variables -----------
static enum { charge_forward, charge_dump, charge_reverse, charge_turn,
        charge_avoidtape} chargeState = charge_forward;
static enum { charge_none, charge_lostbeacon, charge_hit, charge_hittape,
        charge_blocked, charge_finished, charge_avoided, charge_reversed } chargeEvent = charge_none;

//-------- AvoidTape state variables -----------
static enum { avoid_transition, avoid_reverse, avoid_turnleft, avoid_turnright,
        avoid_forward, avoid_revturnleft, avoid_bump} avoidState = avoid_transition;
static enum { avoid_none, avoid_goback, avoid_cutright, avoid_goright,
    avoid_goleft, avoid_cutleft, avoid_reversed, avoid_lefted, avoid_righted,
    avoid_forwarded, avoid_timedout, avoid_failed, avoid_revover, avoid_bumped} avoidEvent = avoid_none;

//-------- FindTape state variables -----------
static enum { find_forward, find_turn } findState = find_forward;
static enum { find_none, find_foundfront, find_found, find_timedout } findEvent = find_none;

//-------- FollowTape state variables -----------
static enum { follow_transition, follow_avoidobstacle, follow_hardleft, follow_left,
        follow_hardright, follow_right, follow_forward, follow_acuteleft, follow_searchleft,
        follow_searchright } followState = follow_transition;
static enum { follow_none, follow_hit, follow_rightfrontoff, follow_rightoff,
        follow_leftfrontoff, follow_leftoff, follow_armon, follow_losttape,
        follow_foundisland, follow_foundacute, follow_acuted, follow_avoided, follow_searchfailed,
        follow_lookleft, follow_lookright, follow_foundfront, follow_acutefailed } followEvent = follow_none;

//-------- AvoidObstacle state variables -----------
static enum { obstacle_reverse, obstacle_turnleft, obstacle_turnright, obstacle_forward }
        obstacleState = obstacle_reverse;
static enum { obstacle_none, obstacle_reversed, obstacle_lefted, obstacle_forwardfailed, obstacle_forwarded,
        obstacle_rightfailed, obstacle_avoided } obstacleEvent = obstacle_none;

static enum { return_left, return_forward, return_avoidobstacle, return_right, return_uturn } returnState = return_left;
static enum { return_none, return_lefted, return_hitobstacle, return_hitwall, return_armon, return_uturned,
        return_righted, return_goright } returnEvent = return_none;



static enum {calibrate_onthreshold, calibrate_offthreshold} calibrateState = calibrate_onthreshold;
static enum { calibrate_none, calibrate_ready, calibrate_timedout, calibrate_next, calibrate_finished } calibrateEvent = calibrate_none;



#define START_STATE follow_tape



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
void DuringReturnIsland();

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

void DuringReturnIslandSM() {
    UpdateReturnIslandEvent();

    switch(returnState) {
        case return_left:
            if (returnEvent == return_lefted)
                InitTimer(TIMER_RETURN, RETURN_COLLIDE_TIMEOUT);
            else
                Drive_Turn(pivot, left, HALF_SPEED);
            break;
        case return_forward:
            if (returnEvent == return_hitobstacle) {
                returnState = return_avoidobstacle;
                InitAvoidObstacleSM();
            }
            else if (returnEvent == return_armon) {
                // EXIT caller picks this up
                Drive_Stop();
            }
            else if (returnEvent == return_hitwall) {
                returnState = return_uturn;
                InitTimer(TIMER_RETURN,RETURN_LEFT_DELAY);
            }
            else if (returnEvent == return_goright) {
                returnState = return_right;
                InitTimer(TIMER_RETURN, RETURN_RIGHT_DELAY);
            }
            else {

                Drive_Forward(MID_SPEED);
            }
            break;
        case return_avoidobstacle:
            DuringAvoidObstacleSM();
            if (avoidEvent == obstacle_avoided) {
                returnState = return_forward;
            }
            break;
        case return_right:
            if (returnEvent == return_righted)
                returnState = return_forward;
            else
                Drive_Turn(soft,right,HALF_SPEED);
            break;
        case return_uturn:
            if (returnEvent == return_uturned)
                returnState = return_forward;
            else
                Drive_Turn(pivot, right, MID_SPEED);
            break;
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
            if (!IsTimerExpired(TIMER_RETURN) && Bumper_AnyTriggered())
                returnEvent = return_hitobstacle;
            else if (Tape_ArmLeftTriggered() || Tape_ArmRightTriggered() || Tape_ArmFrontTriggered())
                returnEvent = return_armon;

            else if (IsTimerExpired(TIMER_RETURN) && Bumper_AnyTriggered())
                returnEvent = return_hitwall;
            else if (Tape_LeftTriggered())
                returnEvent = return_goright;

            break;
        case return_avoidobstacle:
            break;
        case return_right:
            if (IsTimerExpired(TIMER_RETURN))
                returnEvent = return_righted;
            break;
        case return_uturn:
            if (IsTimerExpired(TIMER_RETURN))
                returnEvent == return_uturned;
            break;
    } // switch
}

void DuringAvoidObstacleSM() {
    UpdateAvoidObstacleEvent();

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
            if (obstacleEvent == obstacle_avoided) {
                Drive_Stop();
                // EXIT -- caller picks up
            }
            else {
                Drive_Turn(pivot, right, HALF_SPEED);
            }
            break;
    } // switch
}

void UpdateAvoidObstacleEvent() {
    avoidEvent = avoid_none;

    switch(obstacleState) {
        case obstacle_reverse:
            if (IsTimerExpired(TIMER_MOVE) || Tape_BackTriggered())
                obstacleEvent = obstacle_reversed;
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
            if (obstacleEvent == obstacle_avoided) {
                Drive_Stop();
                // EXIT -- caller picks up
            }
            else {
                Drive_Turn(pivot, right, HALF_SPEED);
            }
            break;
   } // switch
}


/*
 * @Function DuringFollowTapeSM()
 * @remark
 * @date 2012.3.6 08:30 */
void DuringFollowTapeSM() {
    UpdateFollowTapeEvent();
    printf("\nFollow tape STATE=%x EVENT=%x", followState, followEvent);

    switch (followState) {
        case follow_transition:
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

            DuringAvoidObstacleSM();
            if (followEvent == follow_avoided)
                followState = follow_transition;
            break;
        case follow_searchright:
            if (followEvent == follow_searchfailed) {
                followState = follow_searchleft;
            }
            else if (followEvent == follow_foundfront) {
                followState = follow_transition;
                Drive_Stop();
            }
            else {
                Drive_Turn(pivot, right, HALF_SPEED);
            }
            break;
        case follow_searchleft:
            if (followEvent == follow_foundfront)
                followState = follow_transition;
            else
                Drive_Turn(pivot, left,  HALF_SPEED);
            break;
        case follow_acuteleft:
            Drive_Turn(pivot,left,MID_SPEED);
            if (followEvent == follow_acuted)
                followState = follow_transition;
            else if(followEvent == follow_acutefailed)
                followState = follow_transition;
            else
                Drive_Turn(pivot, left, HALF_SPEED);
            break;
        case follow_hardleft:
            Drive_Turn(hard,left,MIN_SPEED);
            followState = follow_transition;
            break;
        case follow_left:
            Drive_Turn(hard,left,HALF_SPEED);
            followState = follow_transition;
            break;
        case follow_hardright:
            Drive_Turn(hard,right,MIN_SPEED);
            followState = follow_transition;
            break;
        case follow_right:
            Drive_Turn(hard,right,HALF_SPEED);
            followState = follow_transition;
            break;
        case follow_forward:
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
            if (Tape_LeftTriggered() && ! Tape_CenterTriggered())
                followEvent = follow_foundacute;
            else if (Tape_LeftTriggered() && Tape_CenterTriggered() && Tape_RightTriggered())
                followEvent = follow_foundisland;
            else if (Bumper_AnyTriggered() && IsTimerExpired(TIMER_START))
                followEvent = follow_hit;
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
            else
                //dbprintf("\nWhat caused this?");
            break;
        case follow_avoidobstacle:
            if (obstacleEvent == obstacle_avoided)
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

    switch (findState) {
        case find_forward:
            if (findEvent == find_foundfront) {
                InitTimer(TIMER_FIND, FIND_TURN_TIMEOUT);
                InitTimer(TIMER_MOVE, FIND_TURN_DELAY);
                findState = find_turn;
            }
            else {
                Drive_Forward(5);
            }
            break;
        case find_turn:

            if (findEvent == find_found) {
                // EXIT -- caller picks up
                Drive_Stop();
            }
            else if (findEvent == find_timedout) {
                findState = find_forward;
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
            /*if (Tape_ArmRightTriggered() || Tape_ArmFrontTriggered() ||
                    Tape_ArmLeftTriggered() )
                findEvent = find_found;
             */

            if (Tape_LeftTriggered() || Tape_CenterTriggered()
                    || Tape_RightTriggered())
                findEvent = find_foundfront;
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
    //dbprintf("\nHandling TargetSM STATE=%u, EVENT=%u", targetState, targetEvent);
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
            if (targetEvent == target_highestpast) {
                targetState = target_returnmax;
                Drive_Turn(pivot, right, FULL_SPEED);
            }
            else if (targetEvent == target_timedout)
                targetState = target_findmaxright;
            else
                Drive_Turn(pivot,left,FULL_SPEED);
            break;
        case target_findmaxright:
            if (targetEvent == target_highestpast) {
                targetState = target_returnmax;
                Drive_Turn(pivot, opposite, FULL_SPEED);
            }
            else
                Drive_Turn(pivot,right,FULL_SPEED);
            break;
        case target_returnmax:
            if (targetEvent == target_foundmax) {
                Drive_Stop();
                // EXIT -- caller picks this up
            }
            break;
        default:
            break;
            //dbprintf("\nHorrible error occured!");

    } // switch

} // DuringTargetSM()


/*
 * @Function UpdateTargetEvent
 * @remark
 * @date 2012.3.6 08:30 */
void UpdateTargetEvent() {
    unsigned int reading = IR_MainReading();
//    targetEvent = target_none;
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
            }
            else if (IsTimerExpired(TIMER_MOVE)) {
                targetEvent = target_timedout;
            }
            else if (reading <(highestIRSeen -50) && highestIRSeen >300) {
                //dbprintf("\nReading %u, highest %u", reading, highestIRSeen);
                // found max and dropping
                targetEvent = target_highestpast;
            }
            break;
        case target_findmaxright:
            if (reading >= highestIRSeen && IR_MainTriggered()) {
                highestIRSeen = reading;
            }
            else if (reading <(highestIRSeen - 50) && highestIRSeen>300) {
                // found max and dropping
                targetEvent = target_highestpast;
            }
            break;
        case target_returnmax:
            if (reading >= (highestIRSeen - 20))
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
    //dbprintf("\nHandling ChargeSM %u", chargeEvent);

    switch (chargeState) {
        case charge_forward:
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
#ifndef DISABLE_AVOID
                Drive_Stop();
                InitAvoidTapeSM();
                chargeState = charge_avoidtape;
#endif
            }
            else {
                Drive_Forward(FULL_SPEED);
            }
            break;
        case charge_avoidtape:
            DuringAvoidTapeSM();
            if (chargeEvent == charge_avoided)
                chargeState = charge_forward;

            break;
        case charge_dump:
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
            if (chargeEvent == charge_finished) {
                // EXIT - caller picks this up
                Drive_Stop();
            }
            else {
                Drive_Turn(hard, left, FULL_SPEED);
            }
            break;

        default:
            break;
           //dbprintf("\nA horrible error occured!");
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
            if (!IR_MainTriggered()) {
            //if (!1) {
                chargeEvent = charge_lostbeacon;
            }
            else if(Tape_AnyTriggered() && IsTimerExpired(TIMER_START)
                && !(IR_MainTriggered() && IR_AngleTriggered()) ) {
                // hit tape, start timer expired,
                // and enemy is not right in front
                chargeEvent = charge_hittape;
            }
            else if(Bumper_AnyTriggered() || IR_AngleTriggered()) {
                chargeEvent = charge_hit;
            }
            break;
        case charge_avoidtape:
            if ( avoidEvent == avoid_forwarded )
                chargeEvent = charge_avoided;
            if (avoidEvent == avoid_bumped)
                chargeEvent = charge_hit;
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
    //dbprintf("\nHandling AvoidSM STATE=%u EVENT=%u", avoidState, avoidEvent);
    switch (avoidState) {
        case avoid_transition:
            ClearTimerExpired(TIMER_MOVE);
            if (avoidEvent == avoid_timedout) {
                // EXIT - caller picks this up
                Drive_Stop();
                ClearTimerExpired(TIMER_AVOID);
            }
            else if (avoidEvent == avoid_goback) {
                InitTimer(TIMER_MOVE, AVOID_REVERSE_DELAY);
                Drive_Reverse(MID_SPEED);
                avoidState = avoid_reverse;
            }
            else if (avoidEvent == avoid_cutleft) {
               Drive_Turn(hard, left, FULL_SPEED);
               avoidState = avoid_turnleft;
            }
            else if (avoidEvent == avoid_goleft) {
                Drive_Turn(soft, left, FULL_SPEED);
                avoidEvent = avoid_none;
                avoidState = avoid_turnleft;
            }
            else if (avoidEvent == avoid_cutright) {
                Drive_Turn(hard, right, FULL_SPEED);
                avoidState = avoid_turnright;
            }
            else if (avoidEvent == avoid_goright) {
                Drive_Turn(soft, right, FULL_SPEED);
                avoidState = avoid_turnright;
            }
            else {
                // avoidEvent == avoid_forwarded
                Drive_Stop();
                // EXIT - caller picks this up
            }
            break;
        case avoid_reverse:
            if(avoidEvent == avoid_reversed) {
                InitTimer(TIMER_MOVE, AVOID_TURN_DELAY);
                Drive_Turn(hard, left, MID_SPEED);
                avoidState = avoid_revturnleft;
            }
            else {

            }
            break;
        case avoid_revturnleft:
            if(avoidEvent == avoid_revover) {
                InitTimer(TIMER_MOVE, AVOID_FORWARD_DELAY);
                avoidState = avoid_forward;
            }
            else {

            }
        case avoid_turnleft:
            if(avoidEvent == avoid_lefted) {
                InitTimer(TIMER_MOVE, AVOID_FORWARD_DELAY);
                avoidState = avoid_forward;
            }
            break;
        case avoid_turnright:
            if(avoidEvent == avoid_righted) {
                InitTimer(TIMER_MOVE, AVOID_FORWARD_DELAY);
                avoidState = avoid_forward;
            }
            break;
        case avoid_forward:
            if (avoidEvent == avoid_forwarded) {
                Drive_Stop();
                // EXIT - caller picks this up
            }
            else if (avoidEvent == avoid_failed) {
                Drive_Stop();
                avoidState = avoid_transition;
            }
            else {
                Drive_Forward(MID_SPEED);
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

    if(Bumper_AnyTriggered())
        avoidState = avoid_bump;

    switch(avoidState){
        case avoid_transition:
            /*
            if ((Tape_LeftTriggered() && Tape_CenterTriggered() && Tape_AnyRightTriggered()) ||
                 (Tape_LeftTriggered() && !Tape_CenterTriggered() && Tape_AnyRightTriggered()) ||
                 (!Tape_LeftTriggered() && Tape_CenterTriggered() && !Tape_AnyRightTriggered())) {
                */
            if (IsTimerExpired(TIMER_AVOID)) {
                avoidEvent = avoid_timedout;
            }
            else if ((Tape_LeftTriggered() && Tape_AnyRightTriggered()) ||
                    (!Tape_LeftTriggered() && !Tape_AnyRightTriggered() && Tape_CenterTriggered()) ) {
                // 111 101 010
                avoidEvent = avoid_goback;
            }
            else if (!Tape_LeftTriggered() && Tape_CenterTriggered() && Tape_AnyRightTriggered()) {
                // 011
                avoidEvent = avoid_cutleft;
            }
            else if (!Tape_LeftTriggered() && !Tape_CenterTriggered() && Tape_AnyRightTriggered()) {
                // 001
                avoidEvent = avoid_goleft;
            }
            else if (Tape_LeftTriggered() && !Tape_CenterTriggered() && !Tape_AnyRightTriggered()) {
                // 100
                avoidEvent = avoid_goright;
            }
            else if (Tape_LeftTriggered() && Tape_CenterTriggered() && !Tape_AnyRightTriggered()) {
                // 110
                avoidEvent = avoid_cutright;
            }
            else {
                // Not sure, just exit state machine
                  avoidEvent = avoid_goback;
            }
            break;
        case avoid_reverse:
            if (IsTimerExpired(TIMER_MOVE) || Tape_BackTriggered()) {
                ClearTimerExpired(TIMER_MOVE);
                avoidEvent = avoid_reversed;
            }
            break;
        case avoid_turnleft:
            if (!Tape_RightTriggered()) {
                avoidEvent = avoid_lefted;
            }
            break;
        case avoid_revturnleft:
            if (IsTimerExpired(TIMER_MOVE)) {
                ClearTimerExpired(TIMER_MOVE);
                avoidEvent = avoid_revover;
            }
            break;
        case avoid_turnright:
            if (!Tape_LeftTriggered())
                avoidEvent = avoid_righted;
            break;
        case avoid_forward:
            if (IsTimerExpired(TIMER_AVOID)) {
                avoidEvent = avoid_forwarded;
            }
            else if (Tape_AnyTriggered()) {
                avoidEvent = avoid_failed;
            }
            else if(IsTimerExpired(TIMER_MOVE)) {
                ClearTimerExpired(TIMER_MOVE);
                avoidEvent = avoid_forwarded;
            }
            break;
        case avoid_bump:
            avoidEvent = avoid_bumped;
            break;
    }//switch

}

/*
  * @Function DuringCalibrateSM()
 * @remark
 * @date  */
void DuringCalibrateSM() {
    UpdateCalibrateEvent();
    //printf("\nHandling CalibrateSM STATE=%u EVENT=%u", calibrateState, calibrateEvent);
    switch (calibrateState) {
        case calibrate_onthreshold:
            if (calibrateEvent == calibrate_next) {
                Tape_SetOnTapeThreshold(CALIBRATE_TAPEHIGHEST_I);
                printf("\nReady for off tape calibration. Trigger front bumper when done.");
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

    // Sub state machines handle topEvents
    switch (topState) {
        case calibrate:
            DuringCalibrateSM();
            //printf("\nHere with event=%d",calibrateEvent);
            if (calibrateEvent == calibrate_timedout
                    || calibrateEvent == calibrate_finished) {
                InitStartState();
                wait();
                InitTimer(TIMER_START, START_DELAY);
            }// if
            break;

        case target:
            DuringTargetSM();
            if (targetEvent == target_foundmax) {
                topState = charge;
                InitChargeSM();
            }

            break;

        case charge:
            DuringChargeSM();
            if (chargeEvent == charge_finished)  {
                // TODO code rest of state machine
                Drive_Stop();
                topState = find_tape;
                InitFindTapeSM();
            }
            else if (chargeEvent == charge_lostbeacon) {
                topState = target;
                InitTargetSM();
            }
            break;
        case find_tape:
            DuringFindTapeSM();
            if (findEvent == find_found) {
                topState = follow_tape;
            }
            break;
        case follow_tape:
            DuringFollowTapeSM();
            if (followEvent == follow_losttape) {
                //topState = find_tape;
                topState = hold;
            }
            else if (followEvent == follow_foundisland) {
                printf("\nHERR!");
                topState = return_island;
                InitReturnIslandSM();
                //topState = return_island;
            }
            break;
            // TODO code rest of state machine

        case return_island:
            //DuringReturnIslandSM();
            // TODO code rest of state machine
            break;
        default:
            break;
          //dbprintf("\nHorrible error occured!");
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

     // Initialize AD system
    /*
    if ( AD_Init(TAPE_LEFT | TAPE_CENTER | TAPE_RIGHT | TAPE_BACK |
        TAPE_ARMFRONT | TAPE_ARMLEFT | TAPE_ARMRIGHT | IR_PINS | BAT_VOLTAGE )
            == SUCCESS) {
        //dbprintf("\nADC initialized successfully.");
    }
    else {
        //dbprintf("\nADC failed to initialize.");
    }
     */
    int adPins = TAPE_LEFT | TAPE_CENTER | TAPE_RIGHT | TAPE_BACK |
     //      TAPE_ARMFRONT | TAPE_ARMLEFT | TAPE_ARMRIGHT;
         TAPE_ARMFRONT | TAPE_ARMLEFT | TAPE_ARMRIGHT | BAT_VOLTAGE | IR_PINS;

    AD_Init(adPins);
    //printf("\nPins=%x", adPins);

    // Initialize modules
    IR_Init();
    Tape_Init();
    Drive_Init();
    Bumper_Init();
    Gate_Init();



    // Initialize state machine

    //topState = target;
    //InitTargetSM();
    //topState = find_tape;
    //InitFindTapeSM();

#ifdef TAPE_CALIBRATE
    InitCalibrateSM();
        
    topState = calibrate;
#else

    InitStartState();
#endif

    InitTimer(TIMER_START, START_DELAY);

    printf("\nReady, set...");

    wait();
    printf(" Go!");


    //dbprintf("\nHello, I am working...");

    // ------------------------------- Main Loop -------------------------------
    while (1) {
        // Handle updates and module state machines
        Tape_HandleSM();
        Drive_Update();
        Bumper_Update();
        IR_Update();

        if (IsTimerExpired(TIMER_START)) {
            HandleTopSM();
        }
        /*
        else if (TAPE_CALIBRATE && Bumper_LeftTriggered() && Bumper_RightTriggered()
                && topState != calibrate) {
            InitCalibrateSM();
            topState = calibrate;
        }
         */


        if (topState == hold) {
                //break;
        }

        //printf("\nTopSTATE! %u", topState);

       while (!IsTransmitEmpty()); // bad, this is blocking code

    }

    Tape_End();
    //Drive_End();
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
    } // switch
}


void InitChargeSM() {
    chargeState = charge_forward;
}

void InitFollowTapeSM() {
    followState = follow_transition;
}

void InitCalibrateSM() {
    calibrateState = calibrate_onthreshold;
    printf("\nReady for on tape threshold calibration. Trigger front bumper when done.");
    InitTimer(TIMER_CALIBRATE, CALIBRATE_TIMEOUT);
    CALIBRATE_INDICATOR_TRIS = 0;
    CALIBRATE_INDICATOR = 0;
}

void InitFindTapeSM() {
    findState = find_forward;
}

void InitAvoidObstacleSM() {
    InitTimer(TIMER_MOVE, OBSTACLE_REVERSE_DELAY);
    obstacleState = obstacle_reverse;
}

void InitReturnIslandSM() {
    returnState = return_left;
    InitTimer(TIMER_MOVE, RETURN_LEFT_DELAY);
}

void InitTargetSM() {

    targetState = target_findmaxleft;
    highestIRSeen = 0;
    /*
    targetState = target_searchleft;
    InitTimer(TIMER_MOVE, TARGET_FIRST_SEARCH_TIME);
     */
    InitTimer(TIMER_MOVE, LEFT_SEARCH_TIME);
}

void InitAvoidTapeSM() {
    avoidState = avoid_transition;
    InitTimer(TIMER_AVOID, AVOID_TIMEOUT);
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

// ---------------------- EOF ----------------------
#endif

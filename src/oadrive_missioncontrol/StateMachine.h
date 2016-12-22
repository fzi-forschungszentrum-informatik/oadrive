// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \author  Vitali Kaiser <vitali.kaiser@live.de>
 * \date    2016-01-15
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_MISSIONCONTROL_STATEMACHINE_H
#define OADRIVE_MISSIONCONTROL_STATEMACHINE_H

#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/deep_history.hpp>
#include <boost/mpl/list.hpp>
#include <boost/shared_ptr.hpp>
#include <exception>

#include <oadrive_world/EventRegion.h>
#include <oadrive_world/Patch.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <oadrive_trafficsign/aadc_roadSign_enums.h>
#include <oadrive_util/Config.h>
#include <oadrive_util/Broker.h>
#include <oadrive_control/DriverModule.h>
#include <oadrive_lanedetection/StreetPatcher.h>
#include <oadrive_obstacle/ProcessUS.h>

#include <iostream>
#include "ManeuverList.h"
#include "IControl4MC.h"

#include "mcLogging.h"
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive{
namespace missioncontrol{

struct SpeedStruct
{
  float SPEED_SUPER_SLOW; //we have to be really slow or something will not work..
  float SPEED_SLOW;
  float SPEED_PARKING;
  float SPEED_NORMAL;
  float SPEED_BOOST;
};

/*float SPEED_SUPERS_SLOW = 0.1; //we have to be really slow or something will not work..
  float SPEED_SLOW = 0.2;
  float SPEED_PARKING = 0.3;
  float SPEED_NORMAL = 0.6;
  float SPEED_BOOST = 1.0;*/

using namespace oadrive::world;
using namespace oadrive::control;
using namespace oadrive::lanedetection;
using namespace oadrive::obstacle;

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

// Forward declare initial state (needed!):
struct StateInit;

// Forward declare all States (not necessarily needed for all states, but
// done so for readability:
struct StateReady;
struct StateParked;
struct StatePullout;
struct StateFollowTrajectory;
struct StateDriving;
struct StateWaitingForTrajectory;
struct StateWaitingForTrajectoryMoving;
struct StateWaitingForTrajectoryHalting;
struct StateSearchParking;
struct StateGoParking;
struct StateCrossing;
struct StateCrossingBlink;
struct StateCrossingHalt;
struct StateCrossingDriving;
struct StateWorking;
struct StateError;
struct StateFinished;
struct StateOvertaking;
struct StateOvertakingHalt;
struct StateOvertakingGo;
struct StateOvertakingBack;
struct StateOvertakingReverse;
struct StateOvertakingSmallForward;
struct StateOvertakingThroughConstruction;
struct StateUTurn;
struct StatePedestrianCrossing;

// Define all possible events:
struct EvGetReady : sc::event< EvGetReady > {};
struct EvJuryGo : sc::event< EvJuryGo > {};
struct EvJuryStop : sc::event<EvJuryStop> {};
struct EvFoundTrajectory : sc::event< EvFoundTrajectory > {};
struct EvTrajectoryEndReached : sc::event< EvTrajectoryEndReached > {};
struct EvTrajectoryEmpty : sc::event< EvTrajectoryEmpty > {};
struct EvTrajectoryReady : sc::event< EvTrajectoryReady > {};
struct EvEnteredBlinkingRegion : sc::event< EvEnteredBlinkingRegion > {
  EventRegionPtr eventRegion;
  EvEnteredBlinkingRegion(EventRegionPtr e) {
    eventRegion = e;
  }
};
struct EvLeftBlinkingRegion : sc::event< EvLeftBlinkingRegion > {};
struct EvEnteredCrossingHaltRegion : sc::event< EvEnteredCrossingHaltRegion > {
  EventRegionPtr eventRegion;
  EvEnteredCrossingHaltRegion(EventRegionPtr e) {
    eventRegion = e;
  }
};
struct EvLeftObstacleRegion : sc::event< EvLeftObstacleRegion >  {
  EventRegionPtr eventRegion;
  EvLeftObstacleRegion(EventRegionPtr e) {
    eventRegion = e;
  }
};
struct EvEnteredObstacleRegion : sc::event< EvEnteredObstacleRegion > {
  EventRegionPtr eventRegion;
  EvEnteredObstacleRegion(EventRegionPtr e) {
    eventRegion = e;
  }
};
struct EvLeftObstaclePassedRegion : sc::event< EvLeftObstaclePassedRegion > {};
struct EvEnteredObstaclePassedRegion : sc::event< EvEnteredObstaclePassedRegion > {};
struct EvLeftCrossingHaltRegion : sc::event< EvLeftCrossingHaltRegion > {};
struct EvEnteredCrossingCenterRegion : sc::event< EvEnteredCrossingCenterRegion > {};
struct EvLeftCrossingCenterRegion : sc::event< EvLeftCrossingCenterRegion > {};
struct EvEnteredParkingSignRegion : sc::event< EvEnteredParkingSignRegion > {
    EventRegionPtr eventRegion;
    EvEnteredParkingSignRegion(EventRegionPtr e) {
      eventRegion = e;
    }
};
struct EvLeftParkingSignRegion : sc::event< EvLeftParkingSignRegion > {};
struct EvEnteredParkingParallelRegion : sc::event< EvEnteredParkingParallelRegion > {};
struct EvLeftParkingParallelRegion : sc::event< EvLeftParkingParallelRegion > {
  EventRegionPtr eventRegion;
  EvLeftParkingParallelRegion(EventRegionPtr e) {
    eventRegion = e;
  }
};
struct EvEnteredParkingCrossRegion : sc::event< EvEnteredParkingCrossRegion > {};
struct EvLeftParkingCrossRegion : sc::event< EvLeftParkingCrossRegion > {
  EventRegionPtr eventRegion;
  EvLeftParkingCrossRegion(EventRegionPtr e) {
    eventRegion = e;
  }
};
struct EvEnteredUnconnectedTrafficSignRegion : sc::event< EvEnteredUnconnectedTrafficSignRegion > {};
struct EvLeftUnconnectedTrafficSignRegion : sc::event< EvLeftUnconnectedTrafficSignRegion > {};

struct EvTimerParking : sc::event< EvTimerParking > {};
struct EvTimerHaltingAtCrossing : sc::event< EvTimerHaltingAtCrossing > {};
struct EvTimerObstacleStop : sc::event< EvTimerObstacleStop > {};
struct EvTimerObstacleCheck : sc::event< EvTimerObstacleCheck > {};
struct EvTimerObstacleRightlane : sc::event< EvTimerObstacleRightlane > {};
struct EvTimerTrajectoryFree : sc::event< EvTimerTrajectoryFree > {};
struct EvTimerTrajectoryFreeAgain : sc::event< EvTimerTrajectoryFreeAgain > {};
struct EvTimerDriveWithNoTrajectory : sc::event< EvTimerDriveWithNoTrajectory > {};
struct EvTimerSetSmallForwardTraj : sc::event< EvTimerSetSmallForwardTraj > {};
struct EvEnteredPedestrianCrossingRegion : sc::event< EvEnteredPedestrianCrossingRegion > {};


struct EvUTurn : sc::event< EvUTurn > {};
struct EvSpeedBoost : sc::event< EvSpeedBoost > {};
struct EvSpeedNormal : sc::event< EvSpeedNormal > {};

struct EvStop : sc::event< EvStop > {};
struct EvError : sc::event< EvError > {};
struct EvFinished : sc::event< EvFinished > {};

// Define the state machine:
// The state machine must be informed which state it has to
// enter when the machine is initiated. That's why StateInit is
// passed as the second template parameter.
struct StateMachine : sc::state_machine< StateMachine, StateInit >
{
  StateMachine( IControl4MC* controller, DriverModule* driverModule,
      TrajectoryFactory* trajectoryFactory, StreetPatcher* streetPatcher )
  {
    mController = controller;
    mDriverModule = driverModule;
    mTrajectoryFactory = trajectoryFactory;
    mStreetPatcher = streetPatcher;

    mSpeeds.SPEED_SUPER_SLOW = Config::getDouble( "MissionControl", "SPEED_SUPER_SLOW", 0.1 );
    mSpeeds.SPEED_SLOW = Config::getDouble( "MissionControl", "SPEED_SLOW", 0.2 );
    mSpeeds.SPEED_PARKING = Config::getDouble( "MissionControl", "SPEED_PARKING", 0.3 );
    mSpeeds.SPEED_NORMAL = Config::getDouble( "MissionControl", "SPEED_NORMAL", 0.6 );
    mSpeeds.SPEED_BOOST = Config::getDouble( "MissionControl", "SPEED_BOOST", 1.0 );

    mWaitingRoundsAtCrossing = Config::getInt( "MissionControl", "WAITING_ROUNDS", 10 );

    LOGGING_INFO( mcLogger, "StateMachine using speeds: " << endl <<
        "\tSPEED_SUPER_SLOW: " << mSpeeds.SPEED_SUPER_SLOW << endl <<
        "\tSPEED_SLOW: " << mSpeeds.SPEED_SLOW << endl <<
        "\tSPEED_PARKING: " << mSpeeds.SPEED_PARKING << endl <<
        "\tSPEED_NORMAL: " << mSpeeds.SPEED_NORMAL << endl <<
        "\tSPEED_BOOST: " << mSpeeds.SPEED_BOOST << endl );
    LOGGING_INFO( mcLogger, "Will wait a maximum of: " << mWaitingRoundsAtCrossing <<
        " seconds at crossings." << endl );

    mACCEnabled = Config::getBool( "MissionControl", "ACC", true );
    mOvertakeEnabled = Config::getBool( "MissionControl", "Overtaking", true );
    mOvertakeStopEarly = Config::getBool( "MissionControl", "OvertakingStopEarly", false );
    mUSDisabled = Config::getBool( "MissionControl", "USDisabled", false );
    mOvertakeTemporaryDisabled = false;
    mOvertakeIgnoreWhileDriving = false;
    mHasBackedUp = false;

    mBackedUpOvertakeCount = 0;

    if( !mACCEnabled )
    {
      LOGGING_WARNING( mcLogger, "ACC is disabled globally!" << endl );
    } else {
      LOGGING_INFO( mcLogger, "ACC is enabled globally." << endl );
    }

    if( !mOvertakeEnabled )
    {
      LOGGING_WARNING( mcLogger, "Overtaking is disabled globally!" << endl );
    } else {
      LOGGING_INFO( mcLogger, "Overtaking is enabled globally." << endl );
    }
    if( mUSDisabled )
    {
      LOGGING_WARNING( mcLogger, "US sensors are disabled globally!" << endl );
      Environment::getInstance()->setCurrentUSSensorLimits( LIMIT_DEACTIVATE );
    } else {
      LOGGING_INFO( mcLogger, "US sensores is enabled globally." << endl );
    }
  }
  ~StateMachine() {};

  void setManeuverList( ManeuverListPtr list ) { mManeuverList = list; }

  ManeuverListPtr getManeuverList() { return mManeuverList; }
  enumManeuver getCurrentManeuver() { return mManeuverList->getCurrentManeuver(); }
  enumManeuver getPreviousManeuer() { return mManeuverList->getPreviosManeuver(); }

  IControl4MC* getController() { return mController; }
  DriverModule* getDriverModule() { return mDriverModule; }
  TrajectoryFactory* getTrajectoryFactory() { return mTrajectoryFactory; }
  StreetPatcher* getStreetPatcher() { return mStreetPatcher; }
  SpeedStruct* getSpeeds() { return &mSpeeds; }
  bool isACCEnabled() { return mACCEnabled; }
  bool isOvertakeEnabled() { return mOvertakeEnabled; }
  bool isUSDisabled() { return mUSDisabled; }
  bool isOvertakingStopEarly() { return mOvertakeStopEarly; }

  bool isOvertakeTemporaryDisabled() { return mOvertakeTemporaryDisabled; }
  void setOvertakeTemporaryDisabled(bool overtakeTemporaryDisabled) { mOvertakeTemporaryDisabled = overtakeTemporaryDisabled; }

  bool isOvertakeIgnoreWhileDriving() { return mOvertakeIgnoreWhileDriving; }
  void setOvertakeIgnoreWhileDriving(bool overtakeIgnoreWhileDriving) { mOvertakeIgnoreWhileDriving = overtakeIgnoreWhileDriving; }

  void setCurrentOvertakingPosition( ExtendedPose2d pose ) { mCurrentOvertakingPose = pose; }
  ExtendedPose2d getCurrentOvertakingPosition() { return mCurrentOvertakingPose; }

  unsigned int getWaitingRoundsAtCrossing() { return mWaitingRoundsAtCrossing; }

  bool hasBackedUp() { return mHasBackedUp; }
  void setHasBackedUp(bool hasBackedUp) { mHasBackedUp = hasBackedUp; }

  // This adds an event region to the environment, which is needed for finding out if we've
  // already passed the obstacle. The function also remembers the event region and removes
  // it if this function is called again with a new region.
  void addOvertakingEventRegion( EventRegionPtr region )
  {
    // If we've added a previous event region for overtaking, remove it now:
    if( mCurrentOvertakingEventRegion )
    {
      Environment::getInstance()->removeEventRegion( region );
    }
    Environment::getInstance()->addEventRegion( region );
    mCurrentOvertakingEventRegion = region;
  }
  EventRegionPtr getCurrentOvertakingEventRegion() { return mCurrentOvertakingEventRegion; }

  void setCrossingEventRegion( EventRegionPtr region ) { mCurrentCrossingEventRegion = region; }
  EventRegionPtr getCrossingEventRegion() { return mCurrentCrossingEventRegion; }

  void setBackedUpOvertakeCount( int backedUpOvertakeCount ) { mBackedUpOvertakeCount = backedUpOvertakeCount; }
  int getBackedUpOvertakeCount() { return mBackedUpOvertakeCount; }

  private:
  ManeuverListPtr mManeuverList;
  IControl4MC* mController;
  DriverModule* mDriverModule;
  TrajectoryFactory* mTrajectoryFactory;
  StreetPatcher* mStreetPatcher;
  SpeedStruct mSpeeds;

  bool mACCEnabled;
  bool mOvertakeEnabled;
  bool mOvertakeTemporaryDisabled;
  bool mOvertakeIgnoreWhileDriving;
  bool mUSDisabled;
  bool mHasBackedUp;
  bool mOvertakeStopEarly;

  unsigned int mWaitingRoundsAtCrossing;
  int mBackedUpOvertakeCount;

  // Remember various things from events for the states (because of deep_history):
  ExtendedPose2d mCurrentOvertakingPose;
  EventRegionPtr mCurrentOvertakingEventRegion;
  EventRegionPtr mCurrentCrossingEventRegion;

};

// Main state. As long as there is no error, the SM is in this state.
// The State has sub-states. StateReady is the one to be started with, so pass as third argument.
struct StateWorking : sc::state< StateWorking, StateMachine, StateReady >
{
  StateWorking( my_context ctx ) : my_base( ctx ) {   // entry
    LOGGING_INFO( mcLogger, "[SM] StateWorking entered" << endl); } // entry
  ~StateWorking() {   // exit
    LOGGING_INFO( mcLogger, "[SM] StateWorking exited" << endl); } // exit
  // Transitions:
  typedef mpl::list <
    sc::transition< EvError, StateError >,
    sc::transition< EvFinished, StateFinished >,
    sc::custom_reaction< EvJuryStop >,
    // No matter what child state, go back to speed normal when we receive this event:
    sc::custom_reaction< EvSpeedNormal >
    > reactions;

  sc::result react( const EvJuryStop & )
  {
    // send stop in kuer mode
    if(Broker::isActive()) {
      Broker::getInstance()->publish( CHANNEL_SEND_STATUS, "game_finished");
    }
    

    return transit< StateInit >();
  }
  sc::result react( const EvSpeedNormal & )
  {
    LOGGING_INFO( mcLogger, "Stop boosting." << endl );
    float currentSpeed = context< StateMachine >().getDriverModule()->getTargetSpeed();
    float normalSpeed = context< StateMachine >().getSpeeds()->SPEED_NORMAL;
    if( currentSpeed > normalSpeed )
    {
      context< StateMachine >().getDriverModule()->setTargetSpeed( normalSpeed );
    }
    return discard_event();
  }
};
struct StateError : sc::state< StateError, StateMachine >
{
  StateError( my_context ctx ) : my_base( ctx ) {
    LOGGING_INFO( mcLogger, "[SM] StateError entered" << endl);
    context< StateMachine >().getDriverModule()->halt();
    context< StateMachine >().getController()->setJuryState( stateCar_ERROR, context< StateMachine >().getManeuverList()->getCurrentAbsManeuverID() );
  } // entry
  ~StateError() { LOGGING_INFO( mcLogger, "[SM] StateError exited" << endl); } // exit
  typedef mpl::list <
    sc::transition< EvJuryStop, StateInit >
    > reactions;
};
struct StateFinished : sc::state< StateFinished, StateMachine >
{
  StateFinished( my_context ctx ) : my_base( ctx ) {
    LOGGING_INFO( mcLogger, "[SM] StateFinished entered" << endl);
    context< StateMachine >().getDriverModule()->halt();
    context< StateMachine >().getController()->setJuryState( stateCar_COMPLETE, context< StateMachine >().getManeuverList()->getCurrentAbsManeuverID() );
  } // entry
  ~StateFinished() { LOGGING_INFO( mcLogger, "[SM] StateFinished exited" << endl); } // exit

  typedef mpl::list <
    sc::transition< EvJuryStop, StateInit >
    > reactions;
};


// Define a state:
struct StateInit : sc::state< StateInit, StateMachine >
{
  StateInit( my_context ctx ) : my_base( ctx ) {

    unsigned int manId = context< StateMachine >().getManeuverList()->getCurrentAbsManeuverID();
    LOGGING_INFO( mcLogger, "[SM] StateInit entered with ManeuverID: " << manId << endl);
    context< StateMachine >().getController()->setJuryState(stateCar_STARTUP, manId);

    context< StateMachine >().getDriverModule()->halt();
    context< StateMachine >().getController()->setLights( HEAD_LIGHT, false );
    context< StateMachine >().getController()->setLights( REVERSE_LIGHT, false );
    context< StateMachine >().getController()->setLights( BRAKE_LIGHT, false );
    context< StateMachine >().getController()->setLights( BLINK_RIGHT_LIGHT, false );
    context< StateMachine >().getController()->setLights( BLINK_LEFT_LIGHT, false );
    context< StateMachine >().getController()->setLights( HAZARD_LIGHT, false );


  } // entry
  ~StateInit() { LOGGING_INFO( mcLogger, "[SM] StateInit exited" << endl); } // exit
  // Transitions:
  typedef mpl::list <
    sc::custom_reaction< EvGetReady >
    //sc::transition< EvGetReady, StateWorking >
    > reactions;

  // Guards:
  sc::result react( const EvGetReady & )
  {
    if(context< StateMachine >().getManeuverList()->isDummy() == false) {
      return transit< StateWorking >();
    } else {
      LOGGING_INFO( mcLogger, "[SM] Can not go to ready mode because Maneuverlist ist not set. Please set it!" << endl);
      return discard_event();
    }

  }
};

struct StateReady : sc::state< StateReady, StateWorking >
{
  StateReady( my_context ctx ) : my_base( ctx ) {		// entry
    unsigned int manId = context< StateMachine >().getManeuverList()->getCurrentAbsManeuverID();
    LOGGING_INFO( mcLogger, "[SM] StateReady entered" << endl);

    // Reset the environment and everything else:
    context< StateMachine >().getController()->reset();

    context< StateMachine >().getController()->setJuryState( stateCar_READY, manId);
    Environment::getInstance()->setAccOn(context< StateMachine >().isACCEnabled());
  }
  ~StateReady() { LOGGING_INFO( mcLogger, "[SM] StateReady exited" << endl); } // exit
  // List of possible transitions:
  typedef mpl::list <
    sc::custom_reaction< EvJuryGo >
    > reactions;
  // Guards:
  sc::result react( const EvJuryGo & evt )
  {
    unsigned int manId = context< StateMachine >().getManeuverList()->getCurrentAbsManeuverID();
    LOGGING_INFO( mcLogger, "[SM] starting at maneuver with id: " << manId << endl);
    context< StateMachine >().getController()->setJuryState( stateCar_RUNNING, manId);

    if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_PULLOUT_LEFT
        || context< StateMachine >().getCurrentManeuver() == MANEUVER_PULLOUT_RIGHT )
    {
      LOGGING_INFO( mcLogger,
          "[SM] Current maneuver is to pull out (=> go to Parking State)" << endl );

      // manually call transition action here because
      // a) much more readable
      // b) I don't understand how "transit" works.
      return transit< StateParked >();
    }
    else
    {
      // manually call transition action here because
      // a) much more readable
      // b) I don't understand how "transit" works.
      return transit< StateFollowTrajectory >();
    }
  }
};
struct StateParked : sc::state< StateParked, StateWorking >
{
  unsigned long mTimerIDParking;
  StateParked( my_context ctx ) : my_base( ctx ) {
    LOGGING_INFO( mcLogger, "[SM] StateParked entered" << endl);

    // We're in a parking spot. Blink Hazard Lights (see AADC rules):
    context< StateMachine >().getController()->setLights(HAZARD_LIGHT,true);

    // Never search for CROSS_SECTIONs while parking!
    context< StateMachine >().getStreetPatcher()->setSearchCrossSections( false );

    // Wait for 3 seconds (see AADC rules):
    mTimerIDParking = context< StateMachine >().getController()->getTimer()->setTimer(
        3500, TIMER_TYPE_PARKING);

    context< StateMachine >().getDriverModule()->halt();  // just in case
  }
  ~StateParked() {	// exit
    LOGGING_INFO( mcLogger, "[SM] StateParked exited" << endl);
   
    // Clean up timers:
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDParking );
  }
  // Transitions:
  typedef mpl::list <
    sc::custom_reaction< EvTimerParking >
    > reactions;
  sc::result react( const EvTimerParking & )
  {
    LOGGING_INFO( mcLogger, "[SM] StateParking: Reacting to EvTimerParking(). "  <<  context< StateMachine >().getPreviousManeuer() << " , " << context< StateMachine >().getCurrentManeuver() << endl );

    ParkingType parkingSpace = PARKING_TYPE_UNKNOWN;

    // Stop blinking hazard lights:
    context< StateMachine >().getController()->setLights(HAZARD_LIGHT,false);

    // If our previous maneuver was a parking maneuver, we should know what kind
    // of parking lot we're on:
    if ( context< StateMachine >().getPreviousManeuer() == MANEUVER_PARKING_CROSS )
    {
      LOGGING_INFO( mcLogger, "[SM] \tPrevious Maneuver was MANEUVER_PARKING_CROSS" <<
          endl << "[SM] \t\t => We're in a cross parking lot!" << endl );
      parkingSpace = PARKING_TYPE_CROSS;
    }
    else if ( context< StateMachine >().getPreviousManeuer() == MANEUVER_PARKING_PARALLEL ) {
      LOGGING_INFO( mcLogger, "[SM] \tPrevious Maneuver was MANEUVER_PARKING_PARALLEL" <<
          endl << "[SM] \t\t => We're in a parallel parking lot!" << endl );
      parkingSpace = PARKING_TYPE_PARALLEL;
    }

    if( parkingSpace == PARKING_TYPE_UNKNOWN )
    {
      cv::Mat image = context< StateMachine >().getController()->getLastBirdViewImage();
      parkingSpace = context< StateMachine >().getStreetPatcher()->getParkingSpaceDirection(image);

      LOGGING_INFO( mcLogger, "[SM] \tStreetPatcher thinks we're in a " );
      if( parkingSpace == PARKING_TYPE_PARALLEL )
      {
        LOGGING_INFO( mcLogger, "PARKING_TYPE_PARALLEL parking lot." << endl );
      } else if( parkingSpace == PARKING_TYPE_CROSS ) {
        LOGGING_INFO( mcLogger, "PARKING_TYPE_CROSS parking lot." << endl );
      } else {
        LOGGING_INFO( mcLogger, "PARKING_TYPE_UNKNOWN parking lot." << endl );
      }
    }

    context< StateMachine >().getDriverModule()->setTargetSpeed(
        context< StateMachine >().getSpeeds()->SPEED_PARKING );	// in m/s

    // If we can't determine the parking lot type, assume we're in a PAKRING_TYPE_PARALLEL:
    // TODO: Better idea?

    if(parkingSpace == PARKING_TYPE_UNKNOWN)
      parkingSpace = PARKING_TYPE_PARALLEL;

    if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_PULLOUT_RIGHT )
    {
      LOGGING_INFO( mcLogger, "[SM] Next maneuver is a pullout right." << endl );
      if(parkingSpace == PARKING_TYPE_PARALLEL){
        LOGGING_INFO( mcLogger, "[SM]\tParking space is a parallel parking space -> " <<
            " use TRAJ_PULLOUT_PARALLEL." << endl );
        context< StateMachine >().getTrajectoryFactory()
          ->setFixedTrajectory( TRAJ_PULLOUT_PARALLEL );
        context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT,true);

        // "Guess" that the street is next to us:
        ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
        double dx = 1.0;		// 1 meter in front of me
        double dy = PATCH_WIDTHS[STRAIGHT]*0.5 + PATCH_WIDTHS[PARKING]*0.5;
        ExtendedPose2d votePose(
            carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
            carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
            carPose.getYaw() );
        context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose );


      } else {
        LOGGING_INFO( mcLogger, "[SM]\tParking space is a cross parking space -> " <<
            " use TRAJ_PULLOUT_CROSS_RIGHT." << endl );
        context< StateMachine >().getTrajectoryFactory()
          ->setFixedTrajectory( TRAJ_PULLOUT_CROSS_RIGHT );
        context< StateMachine >().getController()->setLights(BLINK_RIGHT_LIGHT,true);

        // "Guess" that the street is infront of us:
        ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
        double dx = PATCH_WIDTHS[STRAIGHT]*0.5 + PATCH_LENGTHS[PARKING]*0.5;
        double dy = 0.0;
        ExtendedPose2d votePose(
            carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
            carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
            carPose.getYaw() + M_PI*0.5 );
        context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose );

      }
      context< StateMachine >().getDriverModule()->setTargetSpeed(
          context< StateMachine >().getSpeeds()->SPEED_PARKING );	// in m/s
      context< StateMachine >().getDriverModule()->drive();
      return transit< StatePullout >();
    } else if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_PULLOUT_LEFT ) {
      LOGGING_INFO( mcLogger, "[SM] Next maneuver is a pullout left." << endl );
      context< StateMachine >().getTrajectoryFactory()
        ->setFixedTrajectory( TRAJ_PULLOUT_CROSS_LEFT );
      context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT,true);
      context< StateMachine >().getDriverModule()->setTargetSpeed(
          context< StateMachine >().getSpeeds()->SPEED_PARKING );	// in m/s
      context< StateMachine >().getDriverModule()->drive();

      // "Guess" that the street is infront of us:
      ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
      double dx = PATCH_WIDTHS[STRAIGHT]*0.5 + PATCH_LENGTHS[PARKING]*0.5;
      double dy = 0.0;
      ExtendedPose2d votePose(
          carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
          carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
          carPose.getYaw() + M_PI*0.5 );
      context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose );

      return transit< StatePullout >();
    } else {
      LOGGING_INFO( mcLogger, "[SM] Timer fired, but the next maneuver is not a pullout. "
          "Will ignore event." << endl );
      return discard_event();
    }
  }
};
struct StatePullout : sc::state< StatePullout, StateWorking >
{
  StatePullout( my_context ctx ) : my_base( ctx ) {	// entry
    LOGGING_INFO( mcLogger, "[SM] StatePullout entered" << endl);

    Environment::getInstance()->setNewAccMinSpeed(0.15);
    Environment::getInstance()->setOverwriteAccMinSpeed(true);

    context< StateMachine >().getDriverModule()->drive();  // just in case
  }
  ~StatePullout() {
    LOGGING_INFO( mcLogger, "[SM] StatePullout exited" << endl);

    Environment::getInstance()->setOverwriteAccMinSpeed(false);

    ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
    double dx = 1.0;		// 1 meter in front of me
    double dy = 0.23;		// 0.23 meters next me
    ExtendedPose2d votePose(
            carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
            carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
            carPose.getYaw() );

    Environment::reset();
    context< StateMachine >().getStreetPatcher()->reset();
    context< StateMachine >().getTrajectoryFactory()->clearOldTraj();

    context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose );

    // Finished maneuver, enter next maneuver:
    ManeuverListPtr manList = context< StateMachine >().getManeuverList();
    manList->increase();
    LOGGING_INFO( mcLogger, "[SM]" << manList->toString() << endl);

    if( manList->isFinished() )
      post_event( EvFinished() );

    context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT,false);
    context< StateMachine >().getController()->setLights(BLINK_RIGHT_LIGHT,false);
  } // exit
  // Transitions:
  typedef mpl::list <
    sc::transition< EvTrajectoryEndReached, StateFollowTrajectory >
    > reactions;
};

struct StateFollowTrajectory : sc::state< StateFollowTrajectory, StateWorking,
  StateDriving, sc::has_deep_history >
{
  StateFollowTrajectory( my_context ctx ) : my_base( ctx ) {
    LOGGING_INFO( mcLogger, "[SM] StateFollowTrajectory entered" << endl);
  }
  ~StateFollowTrajectory() { LOGGING_INFO( mcLogger, "[SM] StateFollowTrajectory exited" << endl); }

  typedef mpl::list <
    sc::transition< EvTrajectoryEndReached, StateWaitingForTrajectory >,
    sc::transition< EvTrajectoryEmpty, StateWaitingForTrajectory >
      > reactions;
};

struct StateDriving : sc::state< StateDriving, StateFollowTrajectory >
{
  StateDriving( my_context ctx ) : my_base( ctx ) {
    LOGGING_INFO( mcLogger, "[SM] StateDriving entered" << endl);
    context< StateMachine >().getTrajectoryFactory()->setGenerateFromPatches();
    context< StateMachine >().getDriverModule()->setTargetSpeed(
        context< StateMachine >().getSpeeds()->SPEED_NORMAL );	// in m/s
    context< StateMachine >().getDriverModule()->drive();

    // reset lights
    context< StateMachine >().getController()->setLights(BLINK_RIGHT_LIGHT, false);
    context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT, false);
    context< StateMachine >().getController()->setLights(REVERSE_LIGHT, false);
    context< StateMachine >().getController()->setLights(BRAKE_LIGHT, false);

    if(!context< StateMachine >().isOvertakeIgnoreWhileDriving()) {
      context<StateMachine>().setOvertakeTemporaryDisabled(false);
    }
    context< StateMachine >().setOvertakeIgnoreWhileDriving(false);



    // Search for CROSS_SECTIONs while driving:
    context< StateMachine >().getStreetPatcher()->setSearchCrossSections( true );

    //Go directly into search parking mode, if we shall park next (since we directly have to park) or
    if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_PARKING_CROSS || context< StateMachine >().getCurrentManeuver() == MANEUVER_PARKING_PARALLEL ){
      LOGGING_INFO( mcLogger, "[SM] \tNext maneuver is parking maneuver. Start searching for parking lots." << endl );
      post_event( EvEnteredParkingSignRegion(EventRegionPtr()) );
    }
    // If we've already found a parking sign region, run the event again:
    // (This can happen when we've just finished crossing, but there's already a
    // parking sign ahead).
    if( Environment::getInstance()->isOnParkSignEventRegion() )
    {
      LOGGING_INFO( mcLogger, "[SM] \tThere's a parking sign ahead" << endl );
      post_event( EvEnteredParkingSignRegion(EventRegionPtr()) );
    }

    // Set the US sensors to the current state. This disables some sensors and sets
    // maximum viewing distance for others:
    if( ! context< StateMachine >().isUSDisabled() )
    {
      Environment::getInstance()->setCurrentUSSensorLimits( LIMIT_FOR_DRIVING );
    }

  } // entry
  ~StateDriving() { LOGGING_INFO( mcLogger, "[SM] StateDriving exited" << endl); } // exit
  // Transitions:
  typedef mpl::list <
    sc::custom_reaction< EvEnteredObstacleRegion >,
    sc::custom_reaction< EvEnteredParkingSignRegion >,
    sc::custom_reaction< EvEnteredBlinkingRegion >,
    sc::transition< EvEnteredPedestrianCrossingRegion, StatePedestrianCrossing >,
    sc::transition< EvUTurn, StateUTurn >,
    sc::custom_reaction< EvSpeedBoost>
      > reactions;

  // Guards:
  sc::result react( const EvEnteredParkingSignRegion & )
  {
    if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_PARKING_PARALLEL
        || context< StateMachine >().getCurrentManeuver() == MANEUVER_PARKING_CROSS )
    {
      LOGGING_INFO( mcLogger,
          "[SM] Current maneuver is to park (=> search for parking)" << endl );
      return transit< StateSearchParking >();
    }
    else
    {
      LOGGING_INFO( mcLogger,
          "[SM] Found parking area, but the maneuver list doesn't mention it. "
          "Keep driving!" << endl );
      return discard_event();
    }
  }
  sc::result react( const EvEnteredBlinkingRegion & event)
  {
    if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_LEFT ||
        context< StateMachine >().getCurrentManeuver() == MANEUVER_RIGHT ||
        context< StateMachine >().getCurrentManeuver() == MANEUVER_STRAIGHT )
    {
      LOGGING_INFO( mcLogger,
          "[SM] Reached crossing blinking region." << endl );
      return transit< StateCrossing >();
    } else {
      LOGGING_ERROR( mcLogger,
          "[SM] Reached crossing, but the maneuverlist doesn't mention one." << endl );
      // TODO: handle better (?):
      post_event( EvError() );
      return discard_event();
    }
  }
  sc::result react( const EvEnteredObstacleRegion &ev )
  {
    if(!context< StateMachine >().isOvertakeEnabled() || context< StateMachine >().isOvertakeTemporaryDisabled()){
      return discard_event();
    }

    if(( ev.eventRegion->getEventRegionType() == OBSTACLE_REGION && !context< StateMachine >().hasBackedUp() ) ){
      return discard_event();
    }

    EnvObjectPtr objOfInterest = ev.eventRegion->getObjectOfInterest();
    ObstaclePtr obst = boost::static_pointer_cast<Obstacle>( objOfInterest );
    //LOGGING_INFO( mcLogger, "[SM] ObstacleRegion entered" << endl);
    if(obst) {
      if (Environment::getInstance()->isInFrontOfCar(obst) && Environment::getInstance()->isMultiRelevantObstacle(obst) ) {
        LOGGING_INFO(mcLogger, "[SM] Obstacle of interest" << endl);

        context< StateMachine >().setCurrentOvertakingPosition( obst->getPose() );

        return transit<StateOvertaking>();
      } else {
        //LOGGING_INFO(mcLogger, "[SM] Obstacle can be ignored" << endl);
        return discard_event();
      }
    }
    return discard_event();
  }

  sc::result react( const EvSpeedBoost & )
  {
    LOGGING_INFO( mcLogger, "Boosting!" << endl );
    context< StateMachine >().getDriverModule()->setTargetSpeed(
       context< StateMachine >().getSpeeds()->SPEED_BOOST );
    return discard_event();
  }
};


struct StateWaitingForTrajectory : sc::state< StateWaitingForTrajectory, StateWorking,
  StateWaitingForTrajectoryHalting >
{
  StateWaitingForTrajectory( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectory entered" << endl);
    mWaitingTurns = 0;
  }
  ~StateWaitingForTrajectory() {
    LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectory exited" << endl);
  }

  typedef mpl::list <
    sc::transition< EvTrajectoryReady, sc::deep_history< StateDriving > >
    > reactions;

  unsigned int mWaitingTurns;
};

struct StateWaitingForTrajectoryHalting : sc::state< StateWaitingForTrajectoryHalting, StateWaitingForTrajectory >
{
  unsigned long mTimerIDHalting;
  StateWaitingForTrajectoryHalting( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectoryHalting entered" << endl);

    // We don't have a trajectory? This usually means we don't have any more patches
    // infront of us.
    // To sovle the issue, "guess" where the street is and help the StreetPatcher find
    // the street. In this case, the "guess" is placed so that we're probably standing
    // on the right lane of a street:
    ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
    double dx = 1.0;		// 1 meter in front of me
    double dy = 0.23;		// 0.23 meters next me
    ExtendedPose2d votePose(
        carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
        carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
        carPose.getYaw() );
    unsigned int waitingTurns = context<StateWaitingForTrajectory>().mWaitingTurns;
    bool hasFirstAPrioriVote =
        context<StateMachine>().getStreetPatcher()->hasFirstAPrioriVote();

    if( (context< StateMachine >().getPreviousManeuer() != MANEUVER_PULLOUT_LEFT) &&
        (context< StateMachine >().getPreviousManeuer() != MANEUVER_PULLOUT_RIGHT) )
    {
      // Emergency Mode. Set high a-priori vote in all directions:
        context< StateMachine >().getStreetPatcher()->setAPrioriVote(
            votePose, STRENGTH_HIGH, true );
    } else {

      if( ! hasFirstAPrioriVote ) {
        LOGGING_INFO( mcLogger, "[SM]\tAdding high a-priori vote." << endl);
        context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose, STRENGTH_HIGH );
      } else if( waitingTurns == 0 ) {
        LOGGING_INFO( mcLogger, "[SM]\tAdding low a-priori vote." << endl);
        context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose, STRENGTH_LOW );
      } else if( waitingTurns == 1 ) {
        LOGGING_INFO( mcLogger, "[SM]\tAdding medium a-priori vote." << endl);
        context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose, STRENGTH_MEDIUM );
      } else {
        LOGGING_INFO( mcLogger, "[SM]\tAdding high a-priori vote." << endl);
        context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose, STRENGTH_HIGH );
      }
    }

    context< StateMachine >().getTrajectoryFactory()->setGenerateFromPatches();

    mTimerIDHalting = context< StateMachine >().getController()->getTimer()->setTimer(
        2000, TIMER_TYPE_DRIVE_WITHOUT_TRAJECTORY );

    context<StateWaitingForTrajectory>().mWaitingTurns ++;
    //context< StateMachine >().getController()->setLights(HAZARD_LIGHT,true);

    //TODO: Also start timer so that we can drive forward (or circles?) in case we don't
    // find a trajectory after a while?
  }
  ~StateWaitingForTrajectoryHalting() {
    LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectoryHalting exited" << endl);
    //context< StateMachine >().getController()->setLights(HAZARD_LIGHT,false);
    
    // Clean up timers:
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDHalting );
  } // exit
  // Transitions:
  typedef mpl::list <
    sc::transition< EvTimerDriveWithNoTrajectory, StateWaitingForTrajectoryMoving >
    //sc::custom_reaction< EvTimerDriveWithNoTrajectory >
    > reactions;

/*  sc::result react( const EvTimerDriveWithNoTrajectory & )
  {
    // In Kuer mode, don't drive (we'll reset manually!):
    if(context< StateMachine >().getManeuverList()->isLive() == true)
    {
      return discard_event();
    } else {
      return transit< StateWaitingForTrajectoryMoving >();
    }
  }*/
};

struct StateWaitingForTrajectoryMoving : sc::state< StateWaitingForTrajectoryMoving, StateWaitingForTrajectory >
{

  bool mSetTrajAlready;

  StateWaitingForTrajectoryMoving( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectoryMoving entered" << endl);
    LOGGING_INFO( mcLogger, "[SM] \t(moving forward in the hope of finding another trajectory)" << endl);
    
    Environment::reset();
    context< StateMachine >().getStreetPatcher()->reset();
    context< StateMachine >().getTrajectoryFactory()->clearOldTraj();

    mSetTrajAlready = false;

    context< StateMachine >().getController()->getTimer()->setTimer(
            10, TIMER_SET_SMALL_FORWARD_TRAJ);


  }
  ~StateWaitingForTrajectoryMoving() {
    LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectoryMoving exited" << endl);
    //context< StateMachine >().getController()->setLights(HAZARD_LIGHT,false);
  } // exit
  // Transitions:
  typedef mpl::list <
    sc::custom_reaction< EvTrajectoryEndReached >,
    sc::custom_reaction< EvTimerSetSmallForwardTraj >,
    sc::custom_reaction< EvTrajectoryReady >
    > reactions;

    sc::result react( const EvTimerSetSmallForwardTraj & )
    {

      LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectoryMoving frst event reacted" << endl);
      context< StateMachine >().getDriverModule()->setTargetSpeed(
              context< StateMachine >().getSpeeds()->SPEED_SLOW );	// in m/s
      if(context< StateMachine >().getManeuverList()->isLive() == true)
      {
        //Perhaps TODO
        MultiTrajectory emptyTraj;
        Environment::getInstance()->setTrajectory( emptyTraj );
      }
      else {
        context<StateMachine>().getTrajectoryFactory()
                ->setFixedTrajectory(TRAJ_FORWARD_SMALL);
      }
      context< StateMachine >().getDriverModule()->drive();

      mSetTrajAlready = true;

      return discard_event();
    }

    sc::result react( const EvTrajectoryEndReached & )
    {
      if(mSetTrajAlready){
        LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectoryMoving go to Halt again" << endl);
        return transit< StateWaitingForTrajectoryHalting >();
      }
      else{
        LOGGING_INFO( mcLogger, "[SM] StateWaitingForTrajectoryMoving discard" << endl);
        return discard_event();
      }
    }

    sc::result react( const EvTrajectoryReady & )
    {
      return discard_event();
    }

};

struct StateCrossing : sc::state< StateCrossing, StateFollowTrajectory, StateCrossingBlink >
{
  unsigned int rounds; //how often did we sheduled a wait
  StateCrossing( my_context ctx ) : my_base( ctx ) { // entry

    LOGGING_INFO( mcLogger, "[SM] StateCrossing entered" << endl);

    rounds = 0;
    const sc::event_base* event = this->triggering_event();

    EvEnteredBlinkingRegion const* regionEvent = dynamic_cast<EvEnteredBlinkingRegion const*>(event);
    if( regionEvent != 0 )
    {
      // We've entered a new crossing halting region. Remember it!
      context<StateMachine>().setCrossingEventRegion( regionEvent->eventRegion );
    } else {
      // We've not entered a new region, but we're still in the same one.
      LOGGING_INFO( mcLogger, "[SM] Could not cast region event. " <<
          "This means we're coming from the StateWaitingForTrajectory." << endl );

      // If we don't have information about any crossing region, continue driving:
      if( ! context<StateMachine>().getCrossingEventRegion() )
      {
        post_event( EvLeftCrossingCenterRegion() );
      }
    }
  }

  ~StateCrossing() {
    LOGGING_INFO( mcLogger, "[SM] StateCrossing exited" << endl);
    // Stop blinking:
    context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT,false);
    context< StateMachine >().getController()->setLights(BLINK_RIGHT_LIGHT,false);

    // send info in kuer mode
    if(Broker::isActive()) {
      Broker::getInstance()->publish( CHANNEL_SEND_STATUS, "finished_crossing");
    }

  } // exit
  // Transitions:
  typedef mpl::list <
    sc::custom_reaction< EvLeftCrossingCenterRegion >
    > reactions;

  sc::result react( const EvLeftCrossingCenterRegion & )
  {
    // Remove the EventRegions of the current crossing patch:
    EventRegionPtr region = context<StateMachine>().getCrossingEventRegion();
    if( region )
    {
      PatchPtr patch = boost::dynamic_pointer_cast<Patch>(
          region->getObjectOfInterest() );
      if( patch )
      {
        EventRegionPtrList regions = patch->getEventRegions();
        EventRegionPtrList::iterator it;
        for( it = regions.begin(); it != regions.end(); it ++ )
        {
          Environment::getInstance()->removeEventRegion( *it );
          patch->removeEventRegion( *it );
        }
      }
    }

    // Reset the pointer to the event region:
    context<StateMachine>().setCrossingEventRegion( EventRegionPtr() );

    // Finished maneuver, enter next maneuver:
    ManeuverListPtr manList = context< StateMachine >().getManeuverList();
    manList->increase();
    LOGGING_INFO( mcLogger, "[SM]" << manList->toString() << endl);
    if( manList->isFinished() )
      post_event( EvFinished() );

    return transit< StateDriving >();
  }
};

struct StateCrossingBlink : sc::state< StateCrossingBlink, StateCrossing >
{
  StateCrossingBlink( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateCrossingBlink entered" << endl);

    // Check if this has been triggered by entering a blinking region:
    const sc::event_base* event = this->triggering_event();
    EvEnteredBlinkingRegion const* regionEvent =
      dynamic_cast<EvEnteredBlinkingRegion const*>(event);
    if( regionEvent != 0 )
    {
      EventRegionPtr region = regionEvent->eventRegion;
      if( region )
      {
        PatchPtr patch = boost::dynamic_pointer_cast<Patch>(
            region->getObjectOfInterest() );

        if( patch )
        {
          //Check if we have to slow down get slow
          //check the traffic sign :)
          ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
          TrafficSignPtr trafficSign = patch->getCorrespondingTrafficSign(
              carPose );
          if(trafficSign) {
            if(trafficSign->getSignType() == MARKER_ID_GIVEWAY ||
                trafficSign->getSignType() == MARKER_ID_UNMARKEDINTERSECTION) {
              LOGGING_INFO( mcLogger, "[SM] Found GiveWay or RightBeforeLeft so we slow down..." << endl );
              context< StateMachine >().getDriverModule()->setTargetSpeed(
                  context< StateMachine >().getSpeeds()->SPEED_SLOW);
            }
          } else {
            LOGGING_INFO( mcLogger, "[SM] Could not find a traffic sign at the crossing... we slow down the speed." << endl );

            context< StateMachine >().getDriverModule()->setTargetSpeed(
                context< StateMachine >().getSpeeds()->SPEED_SUPER_SLOW);
          }

          // Set the trajectory already:
          if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_LEFT )
          {
            patch->setAction( DD_LEFT );
            LOGGING_INFO( mcLogger, "[SM] Reached blinking region, blinking left" << endl);

            // This shouldn't do anything, but just in case, disable boosting:
            context< StateMachine >().getStreetPatcher()->setBoostCrossSectionFeatures( false );
            // Going left? Then blink!
            context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT,true);
          } else if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_RIGHT ) {
            patch->setAction( DD_RIGHT );
            LOGGING_INFO( mcLogger, "[SM] Reached blinking region, blinking right" << endl);

            // We might be leaving a Roundabout, so stop boosting!
            context< StateMachine >().getStreetPatcher()->setBoostCrossSectionFeatures( false );
            // Going right? Then blink!
            context< StateMachine >().getController()->setLights(BLINK_RIGHT_LIGHT,true);
          } else if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_STRAIGHT ) {
            patch->setAction( DD_STRAIGHT );
            LOGGING_INFO( mcLogger, "[SM] Reached blinking region, straight no blinking." << endl);
            // Going straight? Then don't blink!
          } else {
            LOGGING_ERROR( mcLogger,
                "[SM] Reached blinking reagion, but the maneuverlist doesn't mention "
                "a crossing." << endl );
            // TODO: handle better:
            post_event( EvError() );
          }

          context< StateMachine >().setOvertakeTemporaryDisabled(true);

          // reset votes in kuer mode
          if(Broker::isActive()) {
            Broker::getInstance()->publish( CHANNEL_SEND_STATUS, "reached_crossing");
          }
        }
      }
    }

    // Set the US sensors to the current state. This disables some sensors and sets
    // maximum viewing distance for others:
    if( ! context< StateMachine >().isUSDisabled() )
    {
      Environment::getInstance()->setCurrentUSSensorLimits( LIMIT_FOR_CROSSING );
    }

    context< StateMachine >().getDriverModule()->drive();  // just in case
  }

  ~StateCrossingBlink() { // exit:
    LOGGING_INFO( mcLogger, "[SM] StateCrossingBlink exited" << endl);
  }

  typedef mpl::list <
    sc::transition< EvEnteredCrossingHaltRegion, StateCrossingHalt >
    > reactions;
};

struct StateCrossingHalt : sc::state< StateCrossingHalt, StateCrossing >
{
  unsigned long mTimerIDHalting;

  StateCrossingHalt( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateCrossingHalt entered" << endl);

    mTimerIDHalting = 0;  // 0 is an invalid id

    //Do we really need to stop?
    bool stop = true;

    //check the traffic sign :)
    EventRegionPtr region = context<StateMachine>().getCrossingEventRegion();
    //EventRegionPtr region = context< StateCrossing >().eventRegion;
    if( region )
    {
      PatchPtr patch = boost::dynamic_pointer_cast<Patch>(
          region->getObjectOfInterest() );

      if( patch )
      {
        //TrafficSignPtr trafficSign = context< StateMachine >().getEnvironment()->getTrafficSignAtPatch(boost::static_pointer_cast<Patch>(patch));
        ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
        TrafficSignPtr trafficSign = patch->getCorrespondingTrafficSign( carPose );
        if(trafficSign) {
          if(trafficSign->getSignType() == MARKER_ID_AHEADONLY) {
            patch->setAction( DD_STRAIGHT );
          } else if (trafficSign->getSignType() == MARKER_ID_STOPANDGIVEWAY){
            LOGGING_INFO( mcLogger, "[SM] We have a traffic sign StopAndGiveWay. Halt." << endl );
            //wait longer at a stopping
            context< StateMachine >().getDriverModule()->halt();	// stop the car.
            mTimerIDHalting = context< StateMachine >().getController()->getTimer()->setTimer(
                3000, TIMER_TYPE_HALTING_AT_CROSSING );
            return;
          } else if (trafficSign->getSignType() == MARKER_ID_GIVEWAY){
            LOGGING_INFO( mcLogger, "[SM] We have a traffic sign GiveWay. Slow down." << endl );
            context< StateMachine >().getDriverModule()->setTargetSpeed(
                context< StateMachine >().getSpeeds()->SPEED_SLOW );
          } else if(trafficSign->getSignType() == MARKER_ID_HAVEWAY) {
            stop = false;
          }
        }

      }
    }

    if(stop) {
      context< StateMachine >().getDriverModule()->halt();	// stop the car.
    }

    // wait almost nothing....
    mTimerIDHalting = context< StateMachine >().getController()->getTimer()->setTimer(
        10, TIMER_TYPE_HALTING_AT_CROSSING );

  }
  ~StateCrossingHalt() {
    LOGGING_INFO( mcLogger, "[SM] StateCrossingHalt exited" << endl);
    context< StateMachine >().getDriverModule()->setTargetSpeed(
        context< StateMachine >().getSpeeds()->SPEED_NORMAL);
    context< StateMachine >().getDriverModule()->drive();	// go to the car.

    // Clean up timers:
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDHalting );
  } // exit
  typedef mpl::list <
    sc::custom_reaction< EvTimerHaltingAtCrossing >
    > reactions;

  // Guards:
  sc::result react( const EvTimerHaltingAtCrossing & )
  {
    //if we cant check the crossing we just go!!!!
    EventRegionPtr region = context<StateMachine>().getCrossingEventRegion();
    if( !region )
    {
      LOGGING_ERROR( mcLogger, "[SM] Could not check crossing because it is not defined in StateCrossing. We are going to drive." << endl );
      return transit< StateCrossingDriving >();
    }

    EnvObjectPtr patch;
    PatchPtr crossingPatch;
    try {
      patch = region->getObjectOfInterest();
      crossingPatch = boost::static_pointer_cast<Patch>(patch);
    } catch(...) {
      LOGGING_ERROR( mcLogger, "[SM] Pointer to Crossing invalid! Could not cast to PatchPtr!" << endl );
    }

    if(!crossingPatch) {
      LOGGING_ERROR( mcLogger, "[SM] Could not check crossing because it is not defined in StateCrossing. We are going to drive." << endl );
      return transit< StateCrossingDriving >();
    }


    //TrafficSignPtr trafficSign = context< StateMachine >().getEnvironment()->getTrafficSignAtPatch(crossingPatch);
    ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
    TrafficSignPtr trafficSign = crossingPatch->getCorrespondingTrafficSign( carPose );


    //check if somebody is on the crossing at the moment
    bool crossingClear = Environment::getInstance()->isObstacleFree(patch, DD_NONE, true);
    bool crossingLeftClear = Environment::getInstance()->isObstacleFree(patch, DD_LEFT, true);
    bool crossingRightClear = Environment::getInstance()->isObstacleFree(patch, DD_RIGHT, true);
    bool crossingStraightClear = Environment::getInstance()->isObstacleFree(patch, DD_STRAIGHT, true);
    crossingLeftClear = true;   // Hack becaus we're blind on the left side:
    std::string s = crossingStraightClear ? " " : "x";
    std::string r = crossingRightClear ? " " : "x";
    std::string l = crossingLeftClear ? "_" : "x";
    std::string c = crossingClear ? " " : "x";
    LOGGING_INFO( mcLogger,"[SM] Crossing Status:" << endl );
    LOGGING_INFO( mcLogger,"[SM]    | " << s << "  |" << endl );
    LOGGING_INFO( mcLogger,"[SM] ___| " << s << "  |___" << endl );
    LOGGING_INFO( mcLogger,"[SM]          " << r << r << " "<< endl );
    LOGGING_INFO( mcLogger,"[SM] _" << l << l << "  " << c << c << "  ___" << endl );
    LOGGING_INFO( mcLogger,"[SM]    |    |" << endl );
    LOGGING_INFO( mcLogger,"[SM]    |    |" << endl );
    /*
    LOGGING_INFO( mcLogger,"[SM]    | x  |" << endl );
    LOGGING_INFO( mcLogger,"[SM] ___| x  |___" << endl );
    LOGGING_INFO( mcLogger,"[SM]          xx " << endl );
    LOGGING_INFO( mcLogger,"[SM] _xx  xx ___" << endl );
    LOGGING_INFO( mcLogger,"[SM]    |    |" << endl );
    LOGGING_INFO( mcLogger,"[SM]    |    |" << endl );*/

    drivingDirection crossingDir = crossingPatch->getAction();

    bool rightBeforeLeft = false;

    if(!trafficSign) {
      rightBeforeLeft = true;
    }
    else if(trafficSign->getSignType() == MARKER_ID_UNMARKEDINTERSECTION) {
      rightBeforeLeft = true;
    }
    else if(trafficSign->getSignType() == MARKER_ID_ROUNDABOUT) {
      //rightBeforeLeft = true;
      // Roundabout. We can't check to the left anyways, so GOGOGO!
      // Start boosting halt lines!
      context< StateMachine >().getStreetPatcher()->setBoostCrossSectionFeatures( true );
      return transit< StateCrossingDriving >();
    }

    if(rightBeforeLeft) { //we dont have a traffic sign aka rechts vor links

      LOGGING_INFO( mcLogger, "[SM] Crossing Situation: right before left" << endl);
      switch (crossingDir) {
        case DD_RIGHT:
          if(crossingClear) {
            LOGGING_INFO( mcLogger, "[SM] Going! Nobody on patch." << endl);
            return transit< StateCrossingDriving >();
          }
          break;
        case DD_STRAIGHT:
          if(crossingClear && crossingRightClear) {
            LOGGING_INFO( mcLogger, "[SM] Going! Nobody on patch and nobody on right." << endl);
            return transit< StateCrossingDriving >();
          }
          break;
        case DD_LEFT:
          if(crossingClear && crossingRightClear && crossingStraightClear) {
            LOGGING_INFO( mcLogger, "[SM] Going! Nobody on patch, nobody on right and straight." << endl);
            return transit< StateCrossingDriving >();
          }
          break;
        default:
          //should not happen
          break;
      }
    } else if(trafficSign->getSignType() == MARKER_ID_HAVEWAY) {
      LOGGING_INFO( mcLogger, "[SM] Crossing Situation: HaveWay." << endl);
      if(crossingClear)
      {
        switch (crossingDir) {
          case DD_RIGHT:
            LOGGING_INFO( mcLogger, "[SM] Going! Nobody on patch." << endl);
            return transit< StateCrossingDriving >();
            break;
          case DD_LEFT:
            if(crossingStraightClear)
            {
              LOGGING_INFO( mcLogger, "[SM] Going! Nobody on patch, nobody straight ahead" << endl);
              return transit< StateCrossingDriving >();
            }
            break;
          case DD_STRAIGHT:
            LOGGING_INFO( mcLogger, "[SM] Going! Nobody on patch" << endl);
            return transit< StateCrossingDriving >();
            break;
          default:
            //should not happen
            break;
        }
      }

    } else if(trafficSign->getSignType() == MARKER_ID_STOPANDGIVEWAY || trafficSign->getSignType() == MARKER_ID_GIVEWAY) {
      LOGGING_INFO( mcLogger, "[SM] Crossing Situation: StopAndGiveWay (or just GiveWay)." << endl );
      if( crossingClear && crossingRightClear && crossingLeftClear && crossingStraightClear )
      {
        LOGGING_INFO( mcLogger, "[SM] Going! Nobody on patch, left, right or straight." << endl);
        return transit< StateCrossingDriving >();
      }
    } else {
      LOGGING_ERROR( mcLogger, "[SM] Crossing Situation: We got traffic sign no " << trafficSign->getSignType() << ", but dont handle it!" << endl);
    }


    //there is something in our way :)

    unsigned int waitingRounds = context<StateMachine>().getWaitingRoundsAtCrossing();
    if(context< StateCrossing >().rounds > waitingRounds)
    {
      LOGGING_INFO( mcLogger, "[SM] Crossing blocked. But we waited already long enough." << endl );
      return transit< StateCrossingDriving >();

    } else {
      LOGGING_INFO( mcLogger, "[SM] Crossing blocked. Keep halting." << endl );
      context< StateCrossing >().rounds++;
      mTimerIDHalting = context< StateMachine >().getController()->getTimer()->setTimer(
          1000, TIMER_TYPE_HALTING_AT_CROSSING );

      context< StateMachine >().getDriverModule()->halt();
      return discard_event();
    }
  }
};
struct StateCrossingDriving : sc::state< StateCrossingDriving, StateCrossing >
{
  StateCrossingDriving( my_context ctx ) : my_base( ctx ) {	// entry
    LOGGING_INFO( mcLogger, "[SM] StateCrossingDriving entered" << endl);
    Environment::getInstance()->setNewAccMinSpeed(0.15);
    Environment::getInstance()->setOverwriteAccMinSpeed(true);
    context< StateMachine >().getDriverModule()->setTargetSpeed(
        context< StateMachine >().getSpeeds()->SPEED_NORMAL );
    context< StateMachine >().getDriverModule()->drive();

    // send info in kuer mode
    if(Broker::isActive()) {
      Broker::getInstance()->publish( CHANNEL_SEND_STATUS, "entered_crossing");
    }
    
  }
  ~StateCrossingDriving() {	// exit
    LOGGING_INFO( mcLogger, "[SM] StateCrossingDriving exited" << endl);
    // Stop indication lights:
    context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT,false);
    context< StateMachine >().getController()->setLights(BLINK_RIGHT_LIGHT,false);
    Environment::getInstance()->setOverwriteAccMinSpeed(false);




  }
};

struct StateSearchParking : sc::state< StateSearchParking, StateFollowTrajectory >
{
  StateSearchParking( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateSearchParking entered" << endl);

    // HOTFIX:
    // In case we start inside the event region of a parking lot, we enter this state
    // before ever setting a trajectory.
    // TODO: Solve this by introducing meta-state.
    ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
    double dx = 1.0;		// 1 meter in front of me
    double dy = PATCH_WIDTHS[STRAIGHT]*0.25;		// 0.25 meters next me
    ExtendedPose2d votePose(
        carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
        carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
        carPose.getYaw() );
    context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose );

    LOGGING_INFO( mcLogger, "[SM] Enable StreetPatcher searchForParkinLots." << endl);
    // Let the street patcher search for parking lots from now on:
    context< StateMachine >().getStreetPatcher()->setSearchParkingLots( true );
    context< StateMachine >().getDriverModule()->setTargetSpeed(
        context< StateMachine >().getSpeeds()->SPEED_SLOW );

    Environment::getInstance()->setRememberObstacles( true );

    // Set the US sensors to the current state. This disables some sensors and sets
    // maximum viewing distance for others:
    if( ! context< StateMachine >().isUSDisabled() )
    {
      Environment::getInstance()->setCurrentUSSensorLimits( LIMIT_FOR_SEARCH_PARKING );
    }
    context< StateMachine >().getDriverModule()->drive();

  } // entry
  ~StateSearchParking() {
    // No longer search for parking lots:
    context< StateMachine >().getStreetPatcher()->setSearchParkingLots( false );
    LOGGING_INFO( mcLogger, "[SM] StateSearchParking exited" << endl);

    Environment::getInstance()->setRememberObstacles( false );
  } // exit


  // Transitions:
  typedef mpl::list <
    sc::custom_reaction< EvLeftParkingParallelRegion >,
    sc::custom_reaction< EvLeftParkingCrossRegion >
      > reactions;

  sc::result react( const EvLeftParkingParallelRegion &ev )
  {
    EnvObjectPtr objOfInterest = ev.eventRegion->getObjectOfInterest();
    PatchPtr parkingPatch = boost::static_pointer_cast<Patch>( objOfInterest );
    if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_PARKING_PARALLEL )
    {
      LOGGING_INFO( mcLogger, "[SM] Parking region found (parallel), checking..." << endl );
      if( Environment::getInstance()->isObstacleFree( parkingPatch, false ) )
      {
        LOGGING_INFO( mcLogger, "[SM] \tis free -> park!" << endl );
        context< StateMachine >().getTrajectoryFactory()->setFixedTrajectory(
            TRAJ_PARKING_PARALLEL );
        return transit< StateGoParking >();
      } else {
        LOGGING_INFO( mcLogger, "[SM] \tis blocked!" << endl );
        return discard_event();
      }
    }
    else
    {
      LOGGING_INFO( mcLogger, "[SM] Parking region found (parallel).  Ignoring!" << endl );
      return discard_event();
    }
  }
  sc::result react( const EvLeftParkingCrossRegion &ev )
  {
    EnvObjectPtr objOfInterest = ev.eventRegion->getObjectOfInterest();
    PatchPtr parkingPatch = boost::static_pointer_cast<Patch>( objOfInterest );
    if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_PARKING_CROSS )
    {
      LOGGING_INFO( mcLogger, "[SM] Parking region found (cross), checking..." << endl );
      if( Environment::getInstance()->isObstacleFree( parkingPatch, false ) )
      {
        LOGGING_INFO( mcLogger, "[SM] \tis free -> park!" << endl );
        context< StateMachine >().getTrajectoryFactory()->setFixedTrajectory(
            TRAJ_PARKING_CROSS );
        return transit< StateGoParking >();
      } else {
        LOGGING_INFO( mcLogger, "[SM] \tis blocked!" << endl );
      }
      return discard_event();
    }
    else
    {
      LOGGING_INFO( mcLogger, "[SM] Parking region found (cross). Ignoring!" << endl );
      return discard_event();
    }
  }

};
struct StateGoParking : sc::state< StateGoParking, StateWorking >
{
  StateGoParking( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateGoParking entered" << endl);

    Environment::getInstance()->setNewAccMinSpeed(0.15);
    Environment::getInstance()->setOverwriteAccMinSpeed(true);

    // We've found a parking lot, stop searching:
    context< StateMachine >().getStreetPatcher()->setSearchParkingLots( false );

    // Never search for CROSS_SECTIONs while parking!
    context< StateMachine >().getStreetPatcher()->setSearchCrossSections( false );

    // We can only park right, so blink right:
    context< StateMachine >().getController()->setLights(BLINK_RIGHT_LIGHT, true );

    context< StateMachine >().getDriverModule()->drive();
  } // entry
  ~StateGoParking() {
    LOGGING_INFO( mcLogger, "[SM] StateGoParking exited" << endl);

    Environment::getInstance()->setOverwriteAccMinSpeed(false);

    // Stop indication lights:
    context< StateMachine >().getController()->setLights(BLINK_RIGHT_LIGHT, false );

    // Finished maneuver, enter next maneuver:
    ManeuverListPtr manList = context< StateMachine >().getManeuverList();
    manList->increase();
    LOGGING_INFO( mcLogger, "[SM]" << manList->toString() << endl);
    if( manList->isFinished() )
      post_event( EvFinished() );

  } // exit
  // Transitions:
  typedef mpl::list <
    sc::transition< EvTrajectoryEndReached, StateParked >
    > reactions;
};

struct StateOvertaking : sc::state< StateOvertaking, StateFollowTrajectory, StateOvertakingHalt >
{
  EventRegionPtr eventRegionPassed;
  EventRegionPtr eventRegionArea;
  StateOvertaking( my_context ctx ) : my_base( ctx ) { // entry

    LOGGING_INFO( mcLogger, "[SM] StateOvertaking entered" << endl);

    // Set the US sensors to the current state. This disables some sensors and sets
    // maximum viewing distance for others:
    if( ! context< StateMachine >().isUSDisabled() )
    {
      Environment::getInstance()->setCurrentUSSensorLimits( LIMIT_FOR_OVERTAKING );
    }

    const sc::event_base* event = this->triggering_event();
    EvEnteredObstacleRegion const* regionEvent =
      dynamic_cast<EvEnteredObstacleRegion const*>(event);

    if( regionEvent != 0 )
    {
      // We've entered a new obstacle region. Remember where the obstacle was!
      EventRegionPtr eventRegion = regionEvent->eventRegion;
      context< StateMachine >().setCurrentOvertakingPosition( eventRegion->getPose() );
    } else {
      // we've re-entered the event from a previous overtake (which was only interrupted).
      // Re-use the old pose.
      LOGGING_INFO( mcLogger, "[SM] Could not cast region event. " <<
          "This means we're coming from the StateWaitingForTrajectory." << endl );
    }

    ExtendedPose2d pose = context< StateMachine >().getCurrentOvertakingPosition();
    EventRegionPtr ev1(new EventRegion( OBSTACLE_REGION, pose ,0.25,0.25 ));
    ev1->setX(pose.getX());
    ev1->setY(pose.getY());
    eventRegionArea = ev1;
    EventRegionPtr ev2(new EventRegion(OBSTACLE_PASSED_REGION, pose,3.0,0.3));
    ev2->setX(pose.getX());
    ev2->setY(pose.getY());
    ev2->setYaw( Environment::getInstance()->getCar()->getYaw() );
    eventRegionPassed = ev2;
  }
  ~StateOvertaking() {
    LOGGING_INFO( mcLogger, "[SM] StateOvertaking exited" << endl);
  }

  typedef mpl::list <
    sc::custom_reaction< EvEnteredBlinkingRegion >,
    sc::custom_reaction< EvEnteredParkingSignRegion >
    > reactions;

    sc::result react( const EvEnteredBlinkingRegion &ev ){

      context< StateMachine >().setOvertakeTemporaryDisabled(true);
      context< StateMachine >().setOvertakeIgnoreWhileDriving(true);
      if(ev.eventRegion){
        ev.eventRegion->setIsCarInside(false);
      }
      return transit< StateDriving >();
    }


    sc::result react( const EvEnteredParkingSignRegion &ev ){

      if ( context< StateMachine >().getCurrentManeuver() == MANEUVER_PARKING_PARALLEL
           || context< StateMachine >().getCurrentManeuver() == MANEUVER_PARKING_CROSS ) {
        Environment::getInstance()->resetStreetLane();
        if (ev.eventRegion) {
          ev.eventRegion->setIsCarInside(false);
        }
        return transit<StateDriving>();
      }
      else{
        return discard_event();
      }
    }



};

struct StateOvertakingHalt : sc::state< StateOvertakingHalt, StateOvertaking >
{
  int freeCounter;
  unsigned long mTimerIDChecking;
  unsigned long mTimerIDHalting;

  StateOvertakingHalt( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingHalt entered" << endl);


    int stopTime = 10000;
    if(context< StateMachine >().hasBackedUp() ) {
      context<StateMachine>().getDriverModule()->halt();
      context< StateMachine >().setHasBackedUp(false);
      stopTime = 1000;
    }else if( context< StateMachine >().isOvertakingStopEarly()) {
      context<StateMachine>().getDriverModule()->halt();
      context< StateMachine >().setHasBackedUp(false);
      stopTime = 10000;
    }
    else{
      context< StateMachine >().getDriverModule()->drive();  // just in case
    }

    // disable detection of cross sections
    context< StateMachine >().getStreetPatcher()->setSearchCrossSections( false );

    freeCounter = 0;

    double dist = Environment::getInstance()->getCar()->calcDistTo(
      context< StateMachine >().getCurrentOvertakingPosition() );
    LOGGING_INFO( mcLogger, "[SM][takeover] Distance to Obstacle : "  << dist << endl);

    //Wait for 3 secs and check afterwards if obstacle still exists
    mTimerIDChecking = context< StateMachine >().getController()->getTimer()->setTimer(
        100, TIMER_TYPE_CHECKING_AT_OBSTACLE );
    mTimerIDHalting = context< StateMachine >().getController()->getTimer()->setTimer(
        stopTime, TIMER_TYPE_HALTING_AT_OBSTACLE );
  }

  ~StateOvertakingHalt() {
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingHalt exited" << endl);

    // Clean up timers:
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDChecking );
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDHalting );
  }

  typedef mpl::list <
    sc::custom_reaction< EvLeftObstacleRegion>,
    sc::custom_reaction< EvTimerObstacleStop >,
    sc::custom_reaction< EvTimerObstacleCheck >
      > reactions;


  sc::result react( const EvTimerObstacleCheck &ev )
  {
    if( Environment::getInstance()->isObstacleFree(context< StateOvertaking >().eventRegionArea, true) ) {
      freeCounter++;
      mTimerIDChecking = context< StateMachine >().getController()->getTimer()->setTimer(
          100, TIMER_TYPE_CHECKING_AT_OBSTACLE );
    }

    if(freeCounter >= 2) {
      LOGGING_INFO( mcLogger, "[SM] Obstacle gone, abort overtake !"<< endl);
      return transit< StateDriving >();
    }

    return discard_event();
  }

  sc::result react( const EvTimerObstacleStop &ev )
  {
    if( !Environment::getInstance()->isObstacleFree(context< StateOvertaking >().eventRegionArea, true) ) {
      LOGGING_INFO( mcLogger, "[SM] Obstacle still there, initating overtake , overtake obstacle: "<< endl);

      context< StateMachine >().addOvertakingEventRegion(
          context< StateOvertaking>().eventRegionPassed );
      return transit< StateOvertakingGo >();
    } else {
      return transit< StateDriving >();
    }

    return discard_event();
  }
  sc::result react( const EvLeftObstacleRegion &ev ){
    return discard_event();
  }


};

struct StateOvertakingGo : sc::state< StateOvertakingGo, StateOvertaking >
{

  int stopCounter;
  unsigned long mTimerIDChecking;

  StateOvertakingGo( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingGo entered" << endl);

    stopCounter = 0;

    context< StateMachine >().getController()->setLights( BLINK_LEFT_LIGHT, true );
    mTimerIDChecking = context< StateMachine >().getController()->getTimer()->setTimer(
        250, TIMER_TYPE_CHECK_TRAJECTORY_FREE);
  }

  ~StateOvertakingGo() {
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingGo exited" << endl);
    context< StateMachine >().getController()->setLights( BLINK_LEFT_LIGHT, false );

    // Clean up timer:
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDChecking );
  }

  typedef mpl::list <
    sc::custom_reaction< EvTimerTrajectoryFree >,
    sc::custom_reaction< EvLeftObstaclePassedRegion>
      > reactions;

  sc::result react( const EvTimerTrajectoryFree &ev ){


    if( context< StateMachine >().getBackedUpOvertakeCount() >2){
      context< StateMachine >().setBackedUpOvertakeCount(0);
      return transit<StateOvertakingSmallForward>();
    }

    ObstaclePtrList probstacles;
    bool clear = true;

    PatchPtr nextPatch;
    if(Environment::getInstance()->getPastCarPatches()->size() > 0 ) {
      LOGGING_INFO(mcLogger, "\tIF !!!!!: " << endl);
      PatchPtr curPatch = Environment::getInstance()->getPastCarPatches()->back();
      if (curPatch) {
        nextPatch = curPatch->getChild(Environment::getInstance()->getCar()->getPose().getYaw(), DD_STRAIGHT);
      }
    }else{
      LOGGING_INFO(mcLogger, "\tELSE !!!!!: " << endl);
      PatchPtr curPatch = Environment::getInstance()->getStreet()->front();
      if (curPatch) {
        nextPatch = curPatch->getChild(Environment::getInstance()->getCar()->getPose().getYaw(), DD_STRAIGHT);
      }
    }

    if (nextPatch) {
      double dist = 0.0;

      nextPatch->setSwitchType(1);
      PatchPtr prevPatch = nextPatch;
      PatchPtr child = nextPatch->getChild(Environment::getInstance()->getCar()->getPose().getYaw(), DD_STRAIGHT);
      while (dist < 0.9  && child ){

        dist = Environment::getInstance()->getCar()->calcDistTo(child);
        if(dist < 1.1){

          prevPatch->setSwitchType(0);
          nextPatch->setSwitchType(1);
          prevPatch = nextPatch;
          nextPatch = child;
          child = nextPatch->getChild(Environment::getInstance()->getCar()->getPose().getYaw(), DD_STRAIGHT);

        }
        else{
          break;
        }

      }
    }

    context< StateMachine >().getTrajectoryFactory()->requestUpdate();

    //Problem obstacles

    Environment::getInstance()->getObstacleInTrajectory(probstacles);

    clear = true;
    if(probstacles.size() > 0){
      for (ObstaclePtrList::iterator it = probstacles.begin();it != probstacles.end(); it++) {
        if(Environment::getInstance()->getCar()->calcDistTo((*it)) < 1.75){
          clear = false;
          break;
        }
      }
    }

    if(!clear){
      LOGGING_INFO( mcLogger, "[SM][takeoverGo] Not Clear, I repeat, not clear" << endl);
      if(nextPatch)
        nextPatch->setSwitchType(0);
      context< StateMachine >().getTrajectoryFactory()->clearOldTraj();
      Environment::getInstance()->resetStreetLane();
      stopCounter++;
      context< StateMachine >().getTrajectoryFactory()->requestUpdate();
      mTimerIDChecking = context< StateMachine >().getController()->getTimer()->setTimer(
          250, TIMER_TYPE_CHECK_TRAJECTORY_FREE);

      LOGGING_INFO( mcLogger, "[SM][takeover] stop counter: "  << stopCounter << endl);

      if(stopCounter > 10){
        LOGGING_INFO( mcLogger, "[SM][takeover] Going through Construction site"<< endl);
        return transit<StateOvertakingThroughConstruction>();
        //LOGGING_INFO( mcLogger, "[SM][takeover] Start backing up"<< endl);
        //return transit<StateOvertakingReverse>();
      }

      return discard_event();
    }

    double dist = Environment::getInstance()->getCar()->calcDistTo(context< StateOvertaking >().eventRegionArea);
    LOGGING_INFO( mcLogger, "[SM][takeover] Distance to Obstacle : "  << dist << endl);

    if(dist < 1.25){
      if(nextPatch)
        nextPatch->setSwitchType(0);
      context< StateMachine >().getTrajectoryFactory()->clearOldTraj();
      Environment::getInstance()->resetStreetLane();
      return transit<StateOvertakingReverse>();
    }


    LOGGING_INFO(mcLogger,
        "\tSwitch Patch: " << Environment::getInstance()->getPastCarPatches()->back()->getId() << endl);


    LOGGING_INFO( mcLogger, "[SM][takeoverGo] GoGoGo" << endl);

    context< StateMachine >().getTrajectoryFactory()->clearOldTraj();
    Environment::getInstance()->setNewAccMinSpeed(0.15);
    Environment::getInstance()->setOverwriteAccMinSpeed(true);
    Environment::getInstance()->setRememberObstacles( true );
    //go
    context< StateMachine >().getDriverModule()->setTargetSpeed(
        context< StateMachine >().getSpeeds()->SPEED_NORMAL );	// in m/s
    context< StateMachine >().getDriverModule()->drive();

    return discard_event();


  }

  sc::result react( const EvLeftObstaclePassedRegion &ev ){
    return transit< StateOvertakingBack >();
  }
};


struct StateOvertakingBack : sc::state< StateOvertakingBack, StateOvertaking > {

  PatchPtr switchBackPatch;
  int clearCounter;
  unsigned long mTimerIDChecking;
  unsigned long mTimerIDCheckingAgain;
  unsigned long mTimerIDReturn;

  StateOvertakingBack( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingBack entered" << endl);
    context< StateMachine >().getController()->setLights( BLINK_RIGHT_LIGHT, true );
    //    context< StateMachine >().getController()->getTimer()->setTimer(
    //          1000, TIMER_TYPE_RETURN_AFTER_OBSTACLE);
    mTimerIDChecking = context< StateMachine >().getController()->getTimer()->setTimer(
        250, TIMER_TYPE_CHECK_TRAJECTORY_FREE);
    clearCounter = 0;

    context< StateMachine >().setBackedUpOvertakeCount(0);

    context< StateMachine >().getDriverModule()->drive();
  }

  ~StateOvertakingBack() {
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingBack exited" << endl);
    context< StateMachine >().getController()->setLights( BLINK_RIGHT_LIGHT, false );
    Environment::getInstance()->setOverwriteAccMinSpeed(false);
    Environment::getInstance()->setRememberObstacles( false );

    // Clean up timers:
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDChecking );
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDCheckingAgain );
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDReturn );
  }

  typedef mpl::list <
    sc::custom_reaction< EvTimerObstacleRightlane >,
    sc::custom_reaction< EvTimerTrajectoryFree >,
    sc::custom_reaction< EvTimerTrajectoryFreeAgain >
      > reactions;

  sc::result react( const EvTimerTrajectoryFree &ev ){

    LOGGING_INFO( mcLogger, "[SM] StateOvertakingBack reacted" << endl);

    // Get the last patch we've reached:
    PatchPtr curPatch;
    if( Environment::getInstance()->getPastCarPatches()->size() > 0 )
    {
      curPatch = Environment::getInstance()->getPastCarPatches()->back();
    } else {
      LOGGING_WARNING( mcLogger,
          "[SM] No past car patches found. Will use first street patch." << endl);
      if( Environment::getInstance()->getStreet()->size() > 0 )
      {
        curPatch = Environment::getInstance()->getStreet()->front();
      } else {
        LOGGING_WARNING( mcLogger,
            "[SM] No first street patch found." << endl);
      }
    }

    PatchPtr nextPatch = curPatch;
    PatchPtr nextNextPatch;
    if(curPatch) {
      nextPatch = curPatch->getChild(Environment::getInstance()->getCarPose().getYaw(), DD_STRAIGHT);
      if (!nextPatch) {
        nextPatch = curPatch;
      }
      nextPatch->setSwitchType(2);
      LOGGING_INFO( mcLogger, "[SM][takeover] Switch Patch " << nextPatch->getId() << endl);
      nextNextPatch = nextPatch->getChild(Environment::getInstance()->getCarPose().getYaw(), DD_STRAIGHT);
      if(nextNextPatch){
        LOGGING_INFO( mcLogger, "[SM][takeover] There is a next Patch" << endl);
      }

    }

    context< StateMachine >().getTrajectoryFactory()->requestUpdate();

    //Problem obstacles
    ObstaclePtrList probstacles;
    Environment::getInstance()->getObstacleInTrajectory(probstacles);

    bool clear = true;
    if(probstacles.size() > 0){

      for (ObstaclePtrList::iterator it = probstacles.begin();it != probstacles.end(); it++) {
        if(Environment::getInstance()->getCar()->calcDistTo((*it)) < 1.75){
          clear = false;
          break;
        }
      }
    }

    if(clear){

      LOGGING_INFO( mcLogger, "[SM][takeover] Clear probstacles" << endl);
      switchBackPatch = nextPatch;
      int backOnTrackTime = 1000;
      clearCounter = 0;

      context< StateMachine >().getTrajectoryFactory()->clearOldTraj();
      mTimerIDReturn = context< StateMachine >().getController()->getTimer()->setTimer(
          backOnTrackTime, TIMER_TYPE_RETURN_AFTER_OBSTACLE);
      mTimerIDCheckingAgain = context< StateMachine >().getController()->getTimer()->setTimer(
          100, TIMER_TYPE_CHECK_TRAJECTORY_FREE_AGAIN);

    }
    else {

      LOGGING_INFO( mcLogger, "[SM][takeover] Not Clear, I repeat, not clear" << endl);
      if (nextPatch) {
        nextPatch->setSwitchType(0);
        LOGGING_INFO( mcLogger, "[SM][takeover] Switch Revert " << nextPatch->getId() << endl);
      }
      context<StateMachine>().getTrajectoryFactory()->requestUpdate();
      mTimerIDChecking = context<StateMachine>().getController()->getTimer()->setTimer(
          250, TIMER_TYPE_CHECK_TRAJECTORY_FREE);

    }

    return discard_event();
  }

  sc::result react( const EvTimerTrajectoryFreeAgain &ev ){

    ObstaclePtrList probstacles;
    Environment::getInstance()->getObstacleInTrajectory(probstacles);

    bool clear = true;
    if(probstacles.size() > 0){

      for (ObstaclePtrList::iterator it = probstacles.begin();it != probstacles.end(); it++) {
        if(Environment::getInstance()->getCar()->calcDistTo((*it)) < 1.75){
          clear = false;
          break;
        }
      }
    }

    if(clear){
      clearCounter++;
      mTimerIDCheckingAgain = context< StateMachine >().getController()->getTimer()->setTimer(
          100, TIMER_TYPE_CHECK_TRAJECTORY_FREE_AGAIN);
      if(clearCounter > 20){
        context< StateMachine >().getTrajectoryFactory()->clearOldTraj();
        return transit< StateDriving >();
      }
      return discard_event();
    }
    else {
      if (switchBackPatch){
        switchBackPatch->setSwitchType(1);
        context< StateMachine >().getTrajectoryFactory()->clearOldTraj();
      }

      context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDReturn );

      context< StateMachine >().getTrajectoryFactory()->requestUpdate();
      mTimerIDChecking = context<StateMachine>().getController()->getTimer()->setTimer(
          250, TIMER_TYPE_CHECK_TRAJECTORY_FREE);
      return discard_event();
    }

    return discard_event();
  }

  sc::result react( const EvTimerObstacleRightlane &ev ){
    return transit< StateDriving >();
  }

};



struct StateOvertakingReverse : sc::state< StateOvertakingReverse, StateOvertaking > {


  StateOvertakingReverse( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingReverse entered" << endl);

    double shouldFit = 1.30;

    double dist = Environment::getInstance()->getCar()->calcDistTo(
        context< StateMachine >().getCurrentOvertakingPosition() );
    double backupDist = 0.30;
    if(dist - shouldFit < -0.1){
      backupDist = shouldFit - dist;
    }

    double scaleFactor = backupDist / 0.30;

    LOGGING_INFO( mcLogger, "[SM] DistanceToObst: "  <<  dist  << endl);
    LOGGING_INFO( mcLogger, "[SM] ToDo backup Distance: "  <<  backupDist  << endl);


    context< StateMachine >().setBackedUpOvertakeCount( context< StateMachine >().getBackedUpOvertakeCount()+1 );

    Environment::getInstance()->setStorePosesBeforePatch(false);

    Environment::getInstance()->setNewAccMinSpeed(0.15);
    Environment::getInstance()->setOverwriteAccMinSpeed(true);
    context< StateMachine >().getTrajectoryFactory()
      ->setFixedTrajectory( TRAJ_BACKUP_OBSTACLE, scaleFactor );
    context< StateMachine >().getDriverModule()->setTargetSpeed(
        context< StateMachine >().getSpeeds()->SPEED_NORMAL );	// in m/s
    context< StateMachine >().getDriverModule()->drive();
    //      context< StateMachine >().getController()->setLights( BLINK_RIGHT_LIGHT, true );

  }

  ~StateOvertakingReverse() {
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingReverse exited" << endl);

    double dist = Environment::getInstance()->getCar()->calcDistTo(context< StateOvertaking >().eventRegionArea);
    LOGGING_INFO( mcLogger, "[SM] DistanceToObst: "  <<  dist  << endl);

    Environment::getInstance()->setStorePosesBeforePatch(true);

    Environment::getInstance()->setOverwriteAccMinSpeed(false);
    Environment::getInstance()->setCarHasBackedUp(true);

    context< StateMachine >().setHasBackedUp(true);

    EventRegionPtr region = context< StateMachine >().getCurrentOvertakingEventRegion();
    if(region){
      region->setIsCarInside(false);
    }
    //      context< StateMachine >().getController()->setLights( BLINK_RIGHT_LIGHT, false );
  }

  typedef mpl::list <
    //            sc::transition< EvTrajectoryEndReached, StateFollowTrajectory >
    sc::transition< EvTrajectoryEndReached, StateDriving >
    > reactions;


};


struct StateOvertakingSmallForward : sc::state< StateOvertakingSmallForward, StateOvertaking > {


    StateOvertakingSmallForward( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingSmallForward entered" << endl);

    Environment::getInstance()->setNewAccMinSpeed(0.15);
    Environment::getInstance()->setOverwriteAccMinSpeed(true);
    context< StateMachine >().getTrajectoryFactory()
            ->setFixedTrajectory( TRAJ_FORWARD_SMALL );
    context< StateMachine >().getDriverModule()->setTargetSpeed(
            context< StateMachine >().getSpeeds()->SPEED_NORMAL );	// in m/s
    context< StateMachine >().getDriverModule()->drive();

  }

  ~StateOvertakingSmallForward() {
    LOGGING_INFO( mcLogger, "[SM] StateOvertakingSmallForward exited" << endl);

    Environment::getInstance()->setOverwriteAccMinSpeed(false);
    ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
    double dx = 1.0;		// 1 meter in front of me
    double dy = 0.23;		// 0.23 meters next me
    ExtendedPose2d votePose(
            carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
            carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
            carPose.getYaw() );

    Environment::reset();
    context< StateMachine >().getStreetPatcher()->reset();
    context< StateMachine >().getTrajectoryFactory()->clearOldTraj();

    context< StateMachine >().getStreetPatcher()->setAPrioriVote( votePose );

  }

  typedef mpl::list <
          sc::transition< EvTrajectoryEndReached, StateDriving >
  > reactions;


};


struct StateOvertakingThroughConstruction : sc::state< StateOvertakingThroughConstruction, StateOvertaking > {


    StateOvertakingThroughConstruction( my_context ctx ) : my_base( ctx ) { // entry
      LOGGING_INFO( mcLogger, "[SM] StateOvertakingThroughConstruction entered" << endl);

      Environment::getInstance()->setNewAccMinSpeed(0.15);
      Environment::getInstance()->setOverwriteAccMinSpeed(true);
      context< StateMachine >().getTrajectoryFactory()
              ->setFixedTrajectory( TRAJ_MIDDLE ); 
      context< StateMachine >().getDriverModule()->setTargetSpeed(
              context< StateMachine >().getSpeeds()->SPEED_NORMAL );	// in m/s
      context< StateMachine >().getDriverModule()->drive();

    }

    ~StateOvertakingThroughConstruction() {
      LOGGING_INFO( mcLogger, "[SM] StateOvertakingThroughConstruction exited" << endl);

      Environment::getInstance()->setOverwriteAccMinSpeed(false);

    }

    typedef mpl::list <
            sc::transition< EvTrajectoryEndReached, StateDriving >
    > reactions;


        };

/*! State for Kuer U-Turn. Drives fixed trajectory, then goes back to driving state. */
struct StateUTurn : sc::state< StateUTurn, StateWorking >
{
  StateUTurn( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StateUTurn entered" << endl);

    // Never search for CROSS_SECTIONs while turning!
    context< StateMachine >().getStreetPatcher()->setSearchCrossSections( false );

    // We can only park right, so blink right:
    context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT, true );

    // Tell the Driver to perform the U-Turn:
    context< StateMachine >().getTrajectoryFactory()
      ->setFixedTrajectory( TRAJ_U_TURN );

    context< StateMachine >().getDriverModule()->drive();
  } // entry
  ~StateUTurn() {
    LOGGING_INFO( mcLogger, "[SM] StateUTurn exited" << endl);

    // Stop indication lights:
    context< StateMachine >().getController()->setLights(BLINK_LEFT_LIGHT, false );

  } // exit
  // Transitions:
  typedef mpl::list <
    sc::transition< EvTrajectoryEndReached, StateDriving >
    > reactions;
};


struct StatePedestrianCrossing : sc::state< StatePedestrianCrossing, StateFollowTrajectory >
{
  unsigned long mTimerIDHalting;
  int numberOfChecks;

  StatePedestrianCrossing( my_context ctx ) : my_base( ctx ) { // entry
    LOGGING_INFO( mcLogger, "[SM] StatePedestrianCrossing entered" << endl);

    mTimerIDHalting = 0;  // 0 is an invalid id

    numberOfChecks = 0;

    context< StateMachine >().getDriverModule()->halt();

    // TODO: enable US

    // wait almost nothing....
    mTimerIDHalting = context< StateMachine >().getController()->getTimer()->setTimer(
        10, TIMER_TYPE_HALTING_AT_CROSSING );

    // Set the US sensors to the current state. This disables some sensors and sets
    // maximum viewing distance for others:
    if( ! context< StateMachine >().isUSDisabled() )
    {
      Environment::getInstance()->setCurrentUSSensorLimits( LIMIT_FOR_PED_CROSSING );
    }
  }
  ~StatePedestrianCrossing() {
    LOGGING_INFO( mcLogger, "[SM] StatePedestrianCrossing exited" << endl);
    context< StateMachine >().getDriverModule()->setTargetSpeed(
        context< StateMachine >().getSpeeds()->SPEED_NORMAL);
    context< StateMachine >().getDriverModule()->drive();	// go to the car.

    // Clean up timers:
    context< StateMachine >().getController()->getTimer()->removeTimer( mTimerIDHalting );
  } // exit
  typedef mpl::list <
    sc::custom_reaction< EvTimerHaltingAtCrossing >
    > reactions;

  sc::result react( const EvTimerHaltingAtCrossing &ev ){

    LOGGING_INFO( mcLogger, "Crosswalk checking: " << numberOfChecks << "/12 times." << endl);
    if( numberOfChecks > 12 )   // This seems like a nice number.
    {
      LOGGING_INFO( mcLogger, "... Enough checks, leaving!" << endl);
      return transit< StateDriving >();
    }


    // Place an event region infront of the car.
    ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
    double dx = ( 0.5 + CAR_ORIGIN_TO_FRONT );
    double dy = PATCH_WIDTHS[STRAIGHT]*0.225;		// 0.25 meters next me
    ExtendedPose2d pose(
        carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
        carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
        carPose.getYaw() );

    EventRegionPtr region( new EventRegion( PED_CROSSING_FREE_REGION, pose, 0.4*2+1.0, 0.8 ) );
    region->setX( pose.getX() );
    region->setY( pose.getY() );
    region->setYaw( pose.getYaw() );
    //Environment::getInstance()->addEventRegion( region );

    if( Environment::getInstance()->isObstacleFree( region, true ) )
    {
      return transit< StateDriving >();
    } else {
      mTimerIDHalting = context< StateMachine >().getController()->getTimer()->setTimer(
          1000, TIMER_TYPE_HALTING_AT_CROSSING );

      numberOfChecks ++;
      return discard_event();
    }
  }
};


}	// namespace
}	// namespace

#endif

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
 * \author  Vitali Kaiser <vitali.kaiser@live.de>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-03
 *
 */
//----------------------------------------------------------------------

#include "MissionControl.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>


//next 3 lines are for Debugging
#include "oadrive_missioncontrol/mcLogging.h"
using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace oadrive::core;
using namespace oadrive::world;
using namespace oadrive::control;
using namespace oadrive::lanedetection;

namespace oadrive{
namespace missioncontrol{

MissionControl::MissionControl( IControl4MC* controller, DriverModule* driverModule,
    TrajectoryFactory* trajectoryFactory, StreetPatcher* streetPatcher )
  : mStateMachine( controller, driverModule, trajectoryFactory, streetPatcher )
  , tf(trajectoryFactory)
  , dm(driverModule)
  , sp(streetPatcher)
{
  this->controller = controller;
  //hier gibt es die unterschiedlichen Debug Levels. Falls du noch einen Stream brauchst muss du ihn in der mcLogging.h und der XML definieren
  //LOGGING_TRACE(mcLogger, "mcLogger Logger is running (Trace)"<<endl);
  //LOGGING_DEBUG(mcLogger, "mcLogger Logger is running.(Debug)"<<endl);
  LOGGING_INFO(mcLogger, "Missioncontrol constructed."<<endl);
  //LOGGING_WARNING(mcLogger,"mcLogger Logger is running (Warning)"<<endl );
  //LOGGING_ERROR(mcLogger, "mcLogger Logger is running (Error)"<<endl);

  m_maneuverList = ManeuverList::getDummy();
  LOGGING_INFO( mcLogger, "Default maneuverlist: " << m_maneuverList->toString() );

  mStateMachine.setManeuverList( m_maneuverList );
  mStateMachine.initiate();

  // Start listening to timer events. From now on, for every timer,
  // my eventTimerFired function will be calle.d
  controller->getTimer()->addListener( this );
}


void MissionControl::eventManeuverListReceived( std::string maneuverList )
{
  LOGGING_INFO(mcLogger, "Maneuverlist received."<<endl);
  m_maneuverList = ManeuverList::parse(maneuverList, this);
  //TODO fire we are ready.

  mStateMachine.setManeuverList( m_maneuverList );
  mStateMachine.process_event( EvGetReady() );
  LOGGING_INFO( mcLogger, "ManeuverList:" << endl << m_maneuverList->toString() );
}


void MissionControl::eventReadyToGeneratePatchTrajectory() {
  LOGGING_INFO(mcLogger, "eventReadyToGenerateTrajectory"<<endl);

  mStateMachine.process_event( EvTrajectoryReady() );


}
/*void MissionControl::eventCannotGeneratePatchTrajectory() {
    LOGGING_TRACE(mcLogger, "eventCannotGeneratePatchTrajectory"<<endl);
}*/

void MissionControl::eventJurySignalReceived (juryActions ja, int maneuverEntryID )
{
  LOGGING_INFO(mcLogger, "eventJurySignalReceived: " << endl <<
               "\tmaneuverID:" << maneuverEntryID << " action: " << ja << endl );

  if(maneuverEntryID < 0) {
    LOGGING_ERROR(mcLogger, "received a negative maneuver id! Cant handle this!!!!!" <<endl);
    return;
  }

  if(((unsigned int)maneuverEntryID) != m_maneuverList->getCurrentAbsManeuverID()) {
    m_maneuverList->setManeuverId(maneuverEntryID);
  }

  switch (ja) {
    case action_GETREADY :
      mStateMachine.process_event( EvGetReady() );
      break;
    case action_START:
      //if(m_maneuverList->setManeuverId(maneuverEntryID) || true) { //TODO remove true
      mStateMachine.process_event( EvJuryGo() );
      /*} else {
                LOGGING_ERROR(mcLogger, "Got wrong entry id!" <<endl);
                        mStateMachine.process_event( EvError() );
        }*/
      break;
    case action_STOP:
      //TODO: send stop to driver module, best in the SM itself
      mStateMachine.process_event( EvJuryStop() );

      break;
  }
}

const int* MissionControl::getCurrentStateId() {
  // TODO: Re-implement for new state machine!!
  return 0;
}


void MissionControl::eventTrajectoryEndReached()
{
  LOGGING_INFO(mcLogger, "eventTrajectoryEndReach"<<endl);
  mStateMachine.process_event( EvTrajectoryEndReached() );
}
void MissionControl::eventTrajectoryEmpty() {

  LOGGING_INFO(mcLogger, "eventTrajectoryEmpty"<<endl);

  mStateMachine.process_event( EvTrajectoryEmpty() );

}

void MissionControl::eventRegionTriggered(EventRegionPtr region, EventType evType ) {

  //LOGGING_INFO(mcLogger, "eventRegion (region type: " << region->getEventRegionType() <<
               //", event Type: " << evType << ")" <<endl);

  if( region->getEventRegionType() == CROSS_SECTION_BLINK )
  {
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredBlinkingRegion(region) );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftBlinkingRegion() );

  } else if( region->getEventRegionType() == CROSS_SECTION_HALT ) {
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredCrossingHaltRegion(region) );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftCrossingHaltRegion() );

  } else if( region->getEventRegionType() == CROSS_SECTION_CENTER ) {
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredCrossingCenterRegion() );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftCrossingCenterRegion() );

  } else if( region->getEventRegionType() == PARKING_SIGN ) {
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredParkingSignRegion(region) );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftParkingSignRegion() );

  } else if( region->getEventRegionType() == PARKING_PARALLEL ) {
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredParkingParallelRegion() );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftParkingParallelRegion( region ) );

  } else if( region->getEventRegionType() == PARKING_CROSS ) {
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredParkingCrossRegion() );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftParkingCrossRegion( region ) );
  } else if(region->getEventRegionType() == OBSTACLE_REGION){
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredObstacleRegion( region ) );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftObstacleRegion(region) );
  } else if(region->getEventRegionType() == OBSTACLE_REGION_SMALL){
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredObstacleRegion( region ) );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftObstacleRegion(region) );
  } else if(region->getEventRegionType() == OBSTACLE_PASSED_REGION){
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredObstaclePassedRegion() );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftObstaclePassedRegion() );
  } else if(region->getEventRegionType() == OVERTAKE_FINISHED_REGION){
     if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvTimerObstacleRightlane() );
  } else if(region->getEventRegionType() == UNCONNECTED_TRAFFIC_SIGN){
    if( evType == EVENT_ENTERED_REGION )
      mStateMachine.process_event( EvEnteredUnconnectedTrafficSignRegion() );
    else if( evType == EVENT_EXITED_REGION )
      mStateMachine.process_event( EvLeftUnconnectedTrafficSignRegion() );

  } else if(region->getEventRegionType() == CROSS_SECTION_OBSTACLES){
    LOGGING_WARNING( mcLogger, "Ignoring event region of CROSS_SECTION_OBSTACLES" << endl );
  } else {
    LOGGING_WARNING( mcLogger, "Don't know how to handle event region type:" <<
                     region->getEventRegionType() << "!" << endl );
  }

}

//TODO: remove
void MissionControl::eventWaitOver() {
  LOGGING_INFO(mcLogger, "eventWaitOver"<<endl);

  //dm->drive();
}

void MissionControl::eventTimerFired( timerType type, unsigned long timerID )
{
  if( type == TIMER_TYPE_HALTING_AT_CROSSING )
  {
    LOGGING_INFO(mcLogger, "Timer fired (Halting at crossing)"<<endl);
    mStateMachine.process_event( EvTimerHaltingAtCrossing() );
  } else if( type == TIMER_TYPE_PARKING )
  {
    LOGGING_INFO(mcLogger, "Timer fired (parking)" <<endl);
    mStateMachine.process_event( EvTimerParking() );
  } else if( type == TIMER_TYPE_HALTING_AT_OBSTACLE )
  {
    LOGGING_INFO(mcLogger, "Timer fired (obstacle stop)" <<endl);
    mStateMachine.process_event( EvTimerObstacleStop() );
  } else if( type == TIMER_TYPE_CHECKING_AT_OBSTACLE )
  {
    LOGGING_INFO(mcLogger, "Timer fired (obstacle check)" <<endl);
    mStateMachine.process_event( EvTimerObstacleCheck() );
  } else if( type == TIMER_TYPE_RETURN_AFTER_OBSTACLE )
  {
    LOGGING_INFO(mcLogger, "Timer fired (return after obstacle)" <<endl);
    mStateMachine.process_event( EvTimerObstacleRightlane() );
  } else if( type == TIMER_TYPE_CHECK_TRAJECTORY_FREE )
  {
    LOGGING_INFO(mcLogger, "Timer fired (check if return traj is free)" <<endl);
    mStateMachine.process_event( EvTimerTrajectoryFree() );
  } else if( type == TIMER_TYPE_CHECK_TRAJECTORY_FREE_AGAIN )
  {
    LOGGING_INFO(mcLogger, "Timer fired (check if return traj is free...again)" <<endl);
    mStateMachine.process_event( EvTimerTrajectoryFreeAgain() );
  } else if( type == TIMER_TYPE_DRIVE_WITHOUT_TRAJECTORY )
  {
    LOGGING_INFO(mcLogger, "Timer fired ( drive without a trajetory)" <<endl);
    mStateMachine.process_event( EvTimerDriveWithNoTrajectory() );
  } else if( type == TIMER_SET_SMALL_FORWARD_TRAJ )
  {
    LOGGING_INFO(mcLogger, "Timer fired ( setting small forward trajetory)" <<endl);
    mStateMachine.process_event( EvTimerSetSmallForwardTraj() );
  } else {
    // ignore
  }
}

void MissionControl::eventSectionCompleted() {
  //ignore
}

void MissionControl::eventParcourFinished() {
  mStateMachine.process_event( EvFinished() );
}

void MissionControl::eventSearchParking() {
  mStateMachine.process_event(EvEnteredParkingSignRegion(EventRegionPtr()));
}

void MissionControl::setJuryState( stateCar state, int manID ) {
  controller->setJuryState(state, manID);
}

void MissionControl::eventUTurn() {
  LOGGING_INFO( mcLogger, "Received U-Turn command." << endl );
  mStateMachine.process_event( EvUTurn() );
}

void MissionControl::setBoostCommand( enumSpeedCommand command )
{
  if( command == SPEED_COMMAND_BOOST )
  {
    LOGGING_INFO( mcLogger, "Received Boost command." << endl );
    mStateMachine.process_event( EvSpeedBoost() );
  } else {
    LOGGING_INFO( mcLogger, "Received Speed Normal command." << endl );
    mStateMachine.process_event( EvSpeedNormal() );
  }
}

}   // namespace
}   // namespace



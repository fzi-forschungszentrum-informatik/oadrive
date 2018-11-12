// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018-09-06
 *
 * State Machine used in the enviroment.
 *
 */
//----------------------------------------------------------------------

#include "StateMachine.h"

StateMachine::StateMachine() {
    mMachine.initiate();
    createTypeMap();
}


void StateMachine::createTypeMap() {
    mStateMap[std::type_index(typeid(Inactive))] = State::INACTIVE;
    mStateMap[std::type_index(typeid(Idle))] = State::IDLING;
    mStateMap[std::type_index(typeid(Parked))] = State::PARKING;
    mStateMap[std::type_index(typeid(NormalDriving))] = State::NORMAL_DRIVING;
    mStateMap[std::type_index(typeid(CarFollowing))] = State::CAR_FOLLOWING;
    mStateMap[std::type_index(typeid(SlowDriving))] = State::SLOW_DRIVING;
    mStateMap[std::type_index(typeid(SlowCarFollowing))] = State::SLOW_CAR_FOLLOWING;
    mStateMap[std::type_index(typeid(Waiting))] = State::WAITING;
    mStateMap[std::type_index(typeid(TeleOp))] = State::TELE_OPERATION;
    mStateMap[std::type_index(typeid(IntersectionLeft))] = State::TURNING_LEFT;
    mStateMap[std::type_index(typeid(IntersectionRight))] = State::TURNING_RIGHT;
    mStateMap[std::type_index(typeid(IntersectionStraight))] = State::DRIVING_STRAIGHT_AT_INTERSECTION;
    mStateMap[std::type_index(typeid(PullOutLeft))] = State::PULLING_OUT_LEFT;
    mStateMap[std::type_index(typeid(PullOutRight))] = State::PULLING_OUT_RIGHT;
    mStateMap[std::type_index(typeid(ParkLeft))] = State::PARKING_LEFT;
    mStateMap[std::type_index(typeid(ParkRight))] = State::PARKING_RIGHT;
    mStateMap[std::type_index(typeid(ZebraCrossing))] = State::WAITING_AT_ZEBRA_CROSSING;
    mStateMap[std::type_index(typeid(SkipZebraCrossing))] = State::DRIVING_THROUGH_ZEBRA;
    mStateMap[std::type_index(typeid(PreBypass))] = State::PRE_BYPASSING;
    mStateMap[std::type_index(typeid(SwitchingForBypass))] = State::SWITCHING_FOR_BYPASS;
    mStateMap[std::type_index(typeid(CurrentlyBypassing))] = State::CURRENTLY_BYPASSING;
    mStateMap[std::type_index(typeid(SwitchingBackForBypass))] = State::SWITCHING_BACK_FOR_BYPASS;
    mStateMap[std::type_index(typeid(PreMerge))] = State::PRE_MERGING;
    mStateMap[std::type_index(typeid(CurrentlyMerging))] = State::CURRENTLY_MERGING;
    mStateMap[std::type_index(typeid(FormingRescueLane))] = State::FORMING_RESCUE_LANE;
    mStateMap[std::type_index(typeid(WaitingInRescueLane))] = State::WAITING_IN_RESCUE_LANE;
    mStateMap[std::type_index(typeid(UnformingRescueLane))] = State::UNFORMING_RESCUE_LANE;
    mStateMap[std::type_index(typeid(DrivingRamp))] = State::DRIVING_RAMP;
}

StateMachine::State StateMachine::queryState() {

    // get pointer to highest level state
    Machine::state_iterator pLeafState = mMachine.state_begin();

    // use type of pointer to infer current level 1 and level 2 state
    State currentState = mStateMap[std::type_index(typeid(*pLeafState))];

    return currentState;
}

void StateMachine::triggerEvent(StateMachine::Event event) {
    switch (event) {
      case Event::JURY_START:
          mMachine.process_event(EvJuryStart());
          break;
      case Event::JURY_PREPARE:
          mMachine.process_event(EvJuryPrepare());
          break;
      case Event::JURY_STOP:
          mMachine.process_event(EvJuryStop());
          break;
      case Event::START:
          mMachine.process_event(EvStart());
          break;
      case Event::STOP:
          mMachine.process_event(EvStop());
          break;
      case Event::RESET:
          mMachine.process_event(EvReset());
          break;
      case Event::ACTIVATE_CAR_FOLLOWING:
          mMachine.process_event(EvActivateCarFollowing());
          break;
      case Event::DEACTIVATE_CAR_FOLLOWING:
          mMachine.process_event(EvDeactivateCarFollowing());
          break;
      case Event::ACTIVATE_SLOW_DRIVING:
          mMachine.process_event(EvActivateSlowDriving());
          break;
      case Event::DEACTIVATE_SLOW_DRIVING:
          mMachine.process_event(EvDeactivateSlowDriving());
          break;
      case Event::START_WAITING:
          mMachine.process_event(EvStartWaiting());
          break;
      case Event::STOP_WAITING:
          mMachine.process_event(EvStopWaiting());
          break;
      case Event::START_TELE_OP:
          mMachine.process_event(EvStartTeleOp());
          break;
      case Event::STOP_TELE_OP:
          mMachine.process_event(EvStopTeleOp());
          break;
      case Event::TURN_LEFT:
          mMachine.process_event(EvTurnLeft());
          break;
      case Event::TURN_RIGHT:
          mMachine.process_event(EvTurnRight());
          break;
      case Event::DRIVE_STRAIGHT:
          mMachine.process_event(EvDriveStraight());
          break;
      case Event::LEAVE_INTERSECTION:
          mMachine.process_event(EvLeaveIntersection());
          break;
      case Event::START_PULLING_OUT_LEFT:
          mMachine.process_event(EvPullOutLeft());
          break;
      case Event::START_PULLING_OUT_RIGHT:
          mMachine.process_event(EvPullOutRight());
          break;
      case Event::START_PARKING_LEFT:
          mMachine.process_event(EvParkLeft());
          break;
      case Event::START_PARKING_RIGHT:
          mMachine.process_event(EvParkRight());
          break;
      case Event::FINISH_PARKING:
          mMachine.process_event(EvFinishParking());  
          break;
      case Event::FINISH_TRAJECTORY:
          mMachine.process_event(EvFinishTrajectory());
          break;
      case Event::APPROACH_ZEBRA_CROSSING:
          mMachine.process_event(EvApproachZebraCrossing());
          break;
      case Event::LEAVE_ZEBRA_CROSSING:
          mMachine.process_event(EvLeaveZebraCrossing());
          break;
      case Event::DRIVE_THROUGH_ZEBRA:
          mMachine.process_event(EvDriveThroughZebraCrossing());
          break;
      case Event::START_BYPASS:
          mMachine.process_event(EvStartBypass());
          break;
      case Event::START_SWITCHING_FOR_BYPASS:
          mMachine.process_event(EvStartSwitching());
          break;
      case Event::FINISH_SWITCHING_FOR_BYPASS:
          mMachine.process_event(EvFinishSwitching());
          break;
      case Event::START_SWITCHING_BACK_FOR_BYPASS:
          mMachine.process_event(EvStartSwitchingBack());
          break;;
      case Event::FINISH_SWITCHING_BACK_FOR_BYPASS:
          mMachine.process_event(EvFinishSwitchingBack());
          break;
      case Event::START_MERGE:
          mMachine.process_event(EvStartMerge());
          break;
      case Event::PERFORM_MERGE:
          mMachine.process_event(EvPerformMerge());
          break;
      case Event::FINISH_MERGE:
          mMachine.process_event(EvFinishMerge());
          break;
      case Event::START_FORMING_RESCUE_LANE:
          mMachine.process_event(EvStartFormingRescueLane());
          break;
      case Event::FINISH_FORMING_RESCUE_LANE:
          mMachine.process_event(EvFinishFormingRescueLane());
          break;
      case Event::START_UNFORMING_RESCUE_LANE:
          mMachine.process_event(EvStartUnformingRescueLane());
          break;
      case Event::FINISH_UNFORMING_RESCUE_LANE:
          mMachine.process_event(EvFinishUnformingRescueLane());
          break;
      case Event::APPROACH_RAMP:
          mMachine.process_event(EvApproachRamp());
          break;
      case Event::LEAVE_RAMP:
          mMachine.process_event(EvLeaveRamp());
          break;
      default:
          break;
    }
}

// for testing
void StateMachine::test() {
    triggerEvent(Event::STOP);
    triggerEvent(Event::START_PULLING_OUT_RIGHT);
    std::cout << queryState() << std::endl;
    triggerEvent(Event::FINISH_TRAJECTORY);
    std::cout << "should not work" << std::endl;
    triggerEvent(Event::START_PULLING_OUT_LEFT);
    std::cout << "nothing should have happened" << std::endl;
    triggerEvent(Event::RESET);
    std::cout << queryState() << std::endl;
    triggerEvent(Event::START_PULLING_OUT_LEFT);
    triggerEvent(Event::RESET);
}

// for testing
int main() {
    StateMachine sm;
    std::cin.get();
    sm.test();
}

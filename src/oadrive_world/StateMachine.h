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

#ifndef OADRIVE_WORLD_STATEMACHINE_H_
#define OADRIVE_WORLD_STATEMACHINE_H_

#include <boost/statechart/transition.hpp>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/mpl/list.hpp>
#include <iostream>
#include <typeinfo>
#include <typeindex>
#include <unordered_map>

#define SHOW_STATE_INFO

namespace sc = boost::statechart;
namespace mpl = boost::mpl;

class StateMachine {

public:

    StateMachine();

    enum State {
        INACTIVE,
        ACTIVE,
        IDLING,
        PARKING,
        NORMAL_DRIVING,
        SLOW_DRIVING,
        CAR_FOLLOWING,
        SLOW_CAR_FOLLOWING,
        WAITING,
        TELE_OPERATION,
        TURNING_LEFT,
        TURNING_RIGHT,
        DRIVING_STRAIGHT_AT_INTERSECTION,
        PULLING_OUT_LEFT,
        PULLING_OUT_RIGHT,
        PARKING_LEFT,
        PARKING_RIGHT,
        WAITING_AT_ZEBRA_CROSSING,
        DRIVING_THROUGH_ZEBRA,
        RESCUE_LANE,
        FORMING_RESCUE_LANE,
        WAITING_IN_RESCUE_LANE,
        UNFORMING_RESCUE_LANE,
        MERGING,
        PRE_MERGING,
        CURRENTLY_MERGING,
        BYPASS,
        PRE_BYPASSING,
        SWITCHING_FOR_BYPASS,
        CURRENTLY_BYPASSING,
        SWITCHING_BACK_FOR_BYPASS,
        DRIVING_RAMP
    };

    enum Event {
        JURY_START,
        JURY_PREPARE,
        JURY_STOP,
        START,
        STOP,
        RESET,
        // normal driving
        ACTIVATE_CAR_FOLLOWING,
        DEACTIVATE_CAR_FOLLOWING,
        ACTIVATE_SLOW_DRIVING,
        DEACTIVATE_SLOW_DRIVING,
        START_WAITING,
        STOP_WAITING,
        // tele op
        START_TELE_OP,
        STOP_TELE_OP,
        // intersections
        TURN_LEFT,
        TURN_RIGHT,
        DRIVE_STRAIGHT,
        LEAVE_INTERSECTION,
        // parking & pulling out
        START_PULLING_OUT_LEFT,
        START_PULLING_OUT_RIGHT,
        START_PARKING_LEFT,
        START_PARKING_RIGHT,
        FINISH_PARKING,
        FINISH_TRAJECTORY,
        // zebra
        APPROACH_ZEBRA_CROSSING,
        DRIVE_THROUGH_ZEBRA,
        LEAVE_ZEBRA_CROSSING,
        // merge
        START_MERGE,
        PERFORM_MERGE,
        FINISH_MERGE,
        // rescue lane
        START_FORMING_RESCUE_LANE,
        FINISH_FORMING_RESCUE_LANE,
        START_UNFORMING_RESCUE_LANE,
        FINISH_UNFORMING_RESCUE_LANE,
        // bypass
        START_BYPASS,
        START_SWITCHING_FOR_BYPASS,
        FINISH_SWITCHING_FOR_BYPASS,
        START_SWITCHING_BACK_FOR_BYPASS,
        FINISH_SWITCHING_BACK_FOR_BYPASS,
        // ramp
        APPROACH_RAMP,
        LEAVE_RAMP
    };

    void test();

    // returns a pair with the current level 1 and level 2 state
    State queryState();

    // triggers an event
    void triggerEvent(Event event);

private:

    // maps types to enums
    std::unordered_map<std::type_index, State> mStateMap;

    // initializes state map
    void createTypeMap();

    // jury events
    struct EvJuryStart : sc::event<EvJuryStart> {};
    struct EvJuryPrepare : sc::event<EvJuryPrepare> {};
    struct EvJuryStop : sc::event<EvJuryStop> {};

    // main control events
    struct EvStart : sc::event<EvStart> {};
    struct EvStop : sc::event<EvStop> {};
    struct EvReset : sc::event<EvReset> {};

    // events for switching lane following modes
    struct EvActivateSlowDriving : sc::event<EvActivateSlowDriving> {};
    struct EvDeactivateSlowDriving : sc::event<EvDeactivateSlowDriving> {};
    struct EvActivateCarFollowing : sc::event<EvActivateCarFollowing> {};
    struct EvDeactivateCarFollowing : sc::event<EvDeactivateCarFollowing> {};
    struct EvStartWaiting : sc::event<EvStartWaiting> {};
    struct EvStopWaiting : sc::event<EvStopWaiting> {};

    // teleop events
    struct EvStartTeleOp : sc::event<EvStartTeleOp> {};
    struct EvStopTeleOp : sc::event<EvStopTeleOp> {};

    // intersection events
    struct EvTurnLeft : sc::event<EvTurnLeft> {};
    struct EvTurnRight : sc::event<EvTurnRight> {};
    struct EvDriveStraight : sc::event<EvDriveStraight> {};
    struct EvLeaveIntersection : sc::event<EvLeaveIntersection> {};

    // pulling out events
    struct EvPullOutLeft : sc::event<EvPullOutLeft> {};
    struct EvPullOutRight : sc::event<EvPullOutRight> {};
    //struct EvFinishPullingOut : sc::event<EvFinishPullingOut> {};

    // parking events
    struct EvParkLeft : sc::event<EvParkLeft> {};
    struct EvParkRight : sc::event<EvParkRight> {};
    struct EvFinishParking : sc::event<EvFinishParking> {};

    // event for finishing driving a fixed trajectory
    struct EvFinishTrajectory : sc::event<EvFinishTrajectory> {};

    // zebra crossing events
    struct EvApproachZebraCrossing : sc::event<EvApproachZebraCrossing> {};
    struct EvDriveThroughZebraCrossing : sc::event<EvDriveThroughZebraCrossing> {};
    struct EvLeaveZebraCrossing : sc::event<EvLeaveZebraCrossing> {};

    // bypass events
    struct EvStartBypass : sc::event<EvStartBypass> {};
    struct EvStartSwitching : sc::event<EvStartSwitching> {}; // not need because of FinishTrajectory event
    struct EvFinishSwitching : sc::event<EvFinishSwitching> {};
    struct EvStartSwitchingBack : sc::event<EvStartSwitchingBack> {};
    struct EvFinishSwitchingBack : sc::event<EvFinishSwitchingBack> {};

    // merge events
    struct EvStartMerge : sc::event<EvStartMerge> {};
    struct EvPerformMerge : sc::event<EvPerformMerge> {};
    struct EvFinishMerge : sc::event<EvFinishMerge> {};

    // rescue lane events
    struct EvStartFormingRescueLane : sc::event<EvStartFormingRescueLane> {};
    struct EvFinishFormingRescueLane : sc::event<EvFinishFormingRescueLane> {};
    struct EvStartUnformingRescueLane : sc::event<EvStartUnformingRescueLane> {};
    struct EvFinishUnformingRescueLane : sc::event<EvFinishUnformingRescueLane> {};

    // ramp events
    struct EvApproachRamp : sc::event<EvApproachRamp> {};
    struct EvLeaveRamp : sc::event<EvLeaveRamp> {};

    ///////////////////////////////////////////////////////////////

    // superstate for inactivity caused by jury actions
    struct Inactive;

    // superstate that contains every other state, allows for easy resetting from every point
    struct Active;

    // actual state machine
    struct Machine : sc::state_machine<Machine, Inactive> {};

    // initial state for Active superstate
    struct Idle;

    //// all other states contained in active

    // state for being parked but not idle
    struct Parked;

    // superstate for lane following
    struct LaneFollowing;
    // substates for lane following
    struct NormalDriving;
    struct SlowDriving;
    struct CarFollowing;
    struct SlowCarFollowing;
    struct Waiting;
    
    // superstate for teleop
    struct TeleOp;

    // superstates for intersections
    struct IntersectionLeft;
    struct IntersectionRight;
    struct IntersectionStraight;

    // superstates for pulling out
    struct PullOutLeft;
    struct PullOutRight;

    // superstates for parking
    struct ParkLeft;
    struct ParkRight;

    // superstate for merging
    struct Merge;

    // substates for merging
    struct PreMerge;
    struct CurrentlyMerging;

    // superstate for bypassing
    struct Bypass;

    // substates for bypassing
    struct PreBypass;
    struct SwitchingForBypass;
    struct CurrentlyBypassing;
    struct SwitchingBackForBypass;

    // superstate for zebra crossing
    struct ZebraCrossing;
    struct SkipZebraCrossing;

    // superstate for rescue lane
    struct RescueLane;

    // substates for rescue lane
    struct FormingRescueLane;
    struct WaitingInRescueLane;
    struct UnformingRescueLane;

    // substates for ramp
    struct DrivingRamp;

    ///////////////////////////////////////////////////////////////

    // Zero level states

    struct Inactive : sc::simple_state<Inactive, Machine> {
        typedef  mpl::list<
           sc::transition<EvJuryPrepare, Active>> reactions;
    };

    struct Active : sc::simple_state<Active, Machine, Idle> {
        typedef mpl::list<
           sc::transition<EvJuryStop, Inactive>,
           sc::transition<EvReset, Active>,
           sc::transition<EvJuryPrepare, Active>> reactions;
    };

    ///////////////////////////////////////////////////////////////

    // First level states

    struct Idle : sc::simple_state<Idle, Active> {

        typedef mpl::list<
            sc::transition<EvStart, LaneFollowing>,
            sc::transition<EvJuryStart, LaneFollowing>,
            sc::transition<EvPullOutLeft, PullOutLeft>,
            sc::transition<EvPullOutRight, PullOutRight>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct Parked : sc::simple_state<Parked, Active> {

        typedef mpl::list<
            sc::transition<EvStop, Idle>,
            sc::transition<EvFinishParking, Idle>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct LaneFollowing : sc::simple_state<LaneFollowing, Active, NormalDriving> {

        typedef mpl::list<
            sc::transition<EvStop, Idle>,
            sc::transition<EvTurnLeft, IntersectionLeft>,
            sc::transition<EvTurnRight, IntersectionRight>,
            sc::transition<EvDriveStraight, IntersectionStraight>,
            sc::transition<EvParkLeft, ParkLeft>,
            sc::transition<EvParkRight, ParkRight>,
            sc::transition<EvApproachZebraCrossing, ZebraCrossing>,
            sc::transition<EvStartBypass, Bypass>,
            sc::transition<EvStartMerge, Merge>,
            sc::transition<EvStartFormingRescueLane, RescueLane>,
            sc::transition<EvDriveThroughZebraCrossing, SkipZebraCrossing>,
            sc::transition<EvApproachRamp, DrivingRamp>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct IntersectionLeft : sc::simple_state<IntersectionLeft, Active> {

        typedef mpl::list<
            sc::transition<EvStartBypass, Bypass>,
            sc::transition<EvLeaveIntersection, LaneFollowing>,
            sc::transition<EvTurnLeft, IntersectionLeft>,
            sc::transition<EvTurnRight, IntersectionRight>,
            sc::transition<EvDriveStraight, IntersectionStraight>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct IntersectionRight : sc::simple_state<IntersectionRight, Active> {

        typedef mpl::list<
            sc::transition<EvStartBypass, Bypass>,
            sc::transition<EvLeaveIntersection, LaneFollowing>,
            sc::transition<EvTurnLeft, IntersectionLeft>,
            sc::transition<EvTurnRight, IntersectionRight>,
            sc::transition<EvDriveStraight, IntersectionStraight>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct IntersectionStraight : sc::simple_state<IntersectionStraight, Active> {
        typedef mpl::list<
            sc::transition<EvStartBypass, Bypass>,
            sc::transition<EvLeaveIntersection, LaneFollowing>,
            sc::transition<EvTurnLeft, IntersectionLeft>,
            sc::transition<EvTurnRight, IntersectionRight>,
            sc::transition<EvDriveStraight, IntersectionStraight>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct TeleOp : sc::simple_state<TeleOp, Active> {
        typedef mpl::list<
            sc::transition<EvStopTeleOp, Idle>> reactions;
    };

    struct PullOutLeft : sc::simple_state<PullOutLeft, Active> {

        typedef mpl::list<
            sc::transition<EvFinishTrajectory, LaneFollowing>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct PullOutRight : sc::simple_state<PullOutRight, Active> {

        typedef mpl::list<
            sc::transition<EvFinishTrajectory, LaneFollowing>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct ParkLeft : sc::simple_state<ParkLeft, Active> {
        typedef  mpl::list<
            sc::transition<EvFinishTrajectory, Parked>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct ParkRight : sc::simple_state<ParkRight, Active> {
        typedef  mpl::list<
            sc::transition<EvFinishTrajectory, Parked>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct Merge : sc::simple_state<Merge, Active, PreMerge> {
        typedef  mpl::list<
            sc::transition<EvFinishMerge, LaneFollowing>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct Bypass : sc::simple_state<Bypass, Active, PreBypass> {
        typedef  mpl::list<
            sc::transition<EvFinishSwitchingBack, LaneFollowing>,
            sc::transition<EvStartMerge, Merge>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct ZebraCrossing : sc::simple_state<ZebraCrossing, Active> {
        typedef mpl::list<
            sc::transition<EvLeaveZebraCrossing, LaneFollowing>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct SkipZebraCrossing : sc::simple_state<SkipZebraCrossing, Active> {
        typedef mpl::list<
            sc::transition<EvLeaveZebraCrossing, LaneFollowing>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct RescueLane : sc::simple_state<RescueLane, Active, FormingRescueLane> {
        typedef  mpl::list<
            sc::transition<EvFinishUnformingRescueLane, LaneFollowing>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    struct DrivingRamp : sc::simple_state<DrivingRamp, Active> {
        typedef mpl::list<
            sc::transition<EvStartMerge, Merge>,
            sc::transition<EvStartTeleOp, TeleOp>> reactions;
    };

    ///////////////////////////////////////////////////////////////

    // Second level states

    // lane following states

    struct NormalDriving : sc::simple_state<NormalDriving, LaneFollowing> {
        typedef  mpl::list<
            sc::transition<EvStartWaiting, Waiting>,
            sc::transition<EvActivateCarFollowing, CarFollowing>,
            sc::transition<EvActivateSlowDriving, SlowDriving>> reactions;
    };

    struct CarFollowing : sc::simple_state<CarFollowing, LaneFollowing> {
        typedef  mpl::list<
            sc::transition<EvStartWaiting, Waiting>,
            sc::transition<EvActivateSlowDriving, SlowCarFollowing>,
            sc::transition<EvDeactivateCarFollowing, NormalDriving>> reactions;
    };

    struct SlowDriving : sc::simple_state<SlowDriving, LaneFollowing> {
        typedef  mpl::list<
            sc::transition<EvStartWaiting, Waiting>,
            sc::transition<EvActivateCarFollowing, SlowCarFollowing>,
            sc::transition<EvDeactivateSlowDriving, NormalDriving>> reactions;
    };

    struct SlowCarFollowing : sc::simple_state<SlowCarFollowing, LaneFollowing> {
        typedef  mpl::list<
            sc::transition<EvStartWaiting, Waiting>,
            sc::transition<EvDeactivateCarFollowing, SlowDriving>,
            sc::transition<EvDeactivateSlowDriving, CarFollowing>> reactions;
    };

    struct Waiting : sc::simple_state<Waiting, LaneFollowing> {
        typedef  mpl::list<
            sc::transition<EvStopWaiting, NormalDriving>> reactions;
    };

    // merging states
    
    struct PreMerge : sc::simple_state<PreMerge, Merge> {
        typedef  mpl::list<
            sc::transition<EvPerformMerge, CurrentlyMerging>> reactions;
    };

    struct CurrentlyMerging : sc::simple_state<CurrentlyMerging, Merge> {
        typedef  mpl::list<
            sc::transition<EvTurnLeft, IntersectionLeft>,
            sc::transition<EvTurnRight, IntersectionRight>,
            sc::transition<EvDriveStraight, IntersectionStraight>> reactions;
    };

    // bypass states

    struct PreBypass : sc::simple_state<PreBypass, Bypass> {
        typedef  mpl::list<
                sc::transition<EvFinishTrajectory, SwitchingForBypass>> reactions;
    };

    struct SwitchingForBypass : sc::simple_state<SwitchingForBypass, Bypass> {
        typedef  mpl::list<
                sc::transition<EvFinishSwitching, CurrentlyBypassing>> reactions;
    };

    struct CurrentlyBypassing : sc::simple_state<CurrentlyBypassing, Bypass> {
        typedef  mpl::list<
                sc::transition<EvStartSwitchingBack, SwitchingBackForBypass>> reactions;
    };

    struct SwitchingBackForBypass : sc::simple_state<SwitchingBackForBypass, Bypass> {
    };

    // rescue lane states

    struct FormingRescueLane : sc::simple_state<FormingRescueLane, RescueLane> {
        typedef  mpl::list<
            sc::transition<EvFinishFormingRescueLane, WaitingInRescueLane>> reactions;
    };

    struct WaitingInRescueLane : sc::simple_state<WaitingInRescueLane, RescueLane> {
        typedef  mpl::list<
            sc::transition<EvStartUnformingRescueLane, UnformingRescueLane>> reactions;
    };

    struct UnformingRescueLane : sc::simple_state<UnformingRescueLane, RescueLane> {
    };

    ///////////////////////////////////////////////////////////////

    Machine mMachine;
};

#endif
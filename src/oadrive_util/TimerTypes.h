// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2017 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-02-01
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_UTIL_TIMERTYPES_H
#define OADRIVE_UTIL_TIMERTYPES_H

namespace oadrive {
namespace util {

enum timerType { TIMER_TYPE_PARKING, TIMER_TYPE_HALTING_AT_CROSSING, TIMER_TYPE_REMOVE_OBSTACLES,
                 TIMER_TYPE_LATERALCONTROLLER , TIMER_TYPE_REMOVE_OLD_OBJECTS,
                 TIMER_TYPE_HALTING_AT_OBSTACLE, TIMER_TYPE_CHECKING_AT_OBSTACLE,
                 TIMER_TYPE_CHECK_TRAJECTORY_FREE, TIMER_TYPE_CHECK_TRAJECTORY_FREE_AGAIN,
                 TIMER_TYPE_RETURN_AFTER_OBSTACLE, TIMER_TYPE_DRIVE_WITHOUT_TRAJECTORY,
                 TIMER_TYPE_SEND_MAP, TIMER_SET_SMALL_FORWARD_TRAJ };

} // namespace
} // namespace

#endif // OADRIVE_UTIL_TIMERTYPES_H

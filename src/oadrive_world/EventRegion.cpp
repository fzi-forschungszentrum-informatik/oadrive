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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-17
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "EventRegion.h"
#include <iostream>
#include <limits>

using namespace oadrive::core;

namespace oadrive {
namespace world {

EventRegion::EventRegion(EventRegionType type,
                         const oadrive::core::Pose2d &pose, const oadrive::core::Pose2d &parkingTakeoff, float width,
                         float height, bool oneTime)
    : EnvObject(ExtendedPose2d(pose), width, height), mParkingTakeoff(parkingTakeoff), mLocalPose(pose), mOneTime(oneTime) {
  mEventRegionType = type;
}

EventRegion::~EventRegion() {}

bool EventRegion::getOneTime() const {
  return mOneTime;
}


void EventRegion::update(EventRegion& region) {
  mEventRegionType = region.getEventRegionType();
  mParkingTakeoff = region.getParkingTakeoff();
  mLocalPose = region.getLocalPose();
}

}  // namespace world
}  // namespace oadrive

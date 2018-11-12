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

#ifndef OADRIVE_WORLD_EVENT_REGION_H
#define OADRIVE_WORLD_EVENT_REGION_H

#include "EnvObject.h"
#include <oadrive_util/CoordinateConverter.h> 
#include <boost/shared_ptr.hpp>
#include <oadrive_core/ExtendedPose2d.h>

namespace oadrive{
namespace world{

const float CROSS_SECTION_HALT_OFFSET = (CAR_ORIGIN_TO_FRONT + 0.16 + 0.05)*2.0;
const float CROSS_SECTION_BLINK_OFFSET = CROSS_SECTION_HALT_OFFSET + 1.0;
const float CROSS_SECTION_CENTER_OFFSET = 0.4*2.0;
const float UNCONNECTED_TRAFFIC_SIGN_REGION_SIZE = 2.8;
const float UNCONNECTED_TRAFFIC_SIGN_OFFSET = UNCONNECTED_TRAFFIC_SIGN_REGION_SIZE*0.5 - 0.1;

enum EventRegionType { CROSS_SECTION_REGION, CROSS_SECTION_HALT, CROSS_SECTION_BLINK, CROSS_SECTION_CENTER, PARKING_SIGN,
                       PARKING_PARALLEL, OBSTACLE_REGION, OBSTACLE_REGION_SMALL, OBSTACLE_PASSED_REGION, OVERTAKE_FINISHED_REGION , PARKING_CROSS, CROSS_SECTION_OBSTACLES,
                       UNCONNECTED_TRAFFIC_SIGN, PED_CROSSING_HALT, PED_CROSSING_FREE_REGION, PARKING_SPACE_REGION };
enum EventType { EVENT_ENTERED_REGION, EVENT_EXITED_REGION };

class EventRegion;	// predefine so shared_ptr can use it
typedef boost::shared_ptr<EventRegion> EventRegionPtr;		// define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<EventRegion> > EventRegionPtrList;

class EventRegion : public EnvObject
{
public:
  EventRegion( EventRegionType mType, const oadrive::core::Pose2d &mPose, const oadrive::core::Pose2d &mParkingTakeoff,
               float mWidth, float mHeight, bool oneTime = false );
  ~EventRegion();

  oadrive::core::Pose2d getLocalPose() const { return mLocalPose; }

  oadrive::core::Pose2d getParkingTakeoff() const { return mParkingTakeoff; }

  EventRegionType getEventRegionType() const { return mEventRegionType; }

  bool getOneTime() const;

  void update(EventRegion& region);

private:

  EventRegionType  mEventRegionType;

  oadrive::core::Pose2d mLocalPose;
  oadrive::core::Pose2d mParkingTakeoff;

  bool mOneTime = false;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}	// namespace
}	// namespace

#endif

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
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-19
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_TRAFFICSIGN_H
#define OADRIVE_WORLD_TRAFFICSIGN_H

#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_util/CoordinateConverter.h>
#include <boost/shared_ptr.hpp>
#include <list>
#include "EnvObject.h"
#include "Obstacle.h"
#include "Patch.h"

namespace oadrive {
namespace world {

//! probability added to each detection of the same traffic sign
const float TRAFFIC_SIGN_PROBABILITY_ADD = 0.1;
//! probability must be greater than set value for a traffic sign to connect to a special patch (0.1
//! means, at least 2 detections)
const float TRAFFIC_SIGN_MINIMUM_CONNECTION_PROBABLITY = 0.1;
//! merging distance for traffic signs in meters
const float TRAFFIC_SIGN_MERGING_DIST = 0.6;
//! Maximum allowed distance between traffic sign and center point of CROSS_SECTION:
const float TRAFFIC_SIGN_MAX_DIST_TO_CROSS_SECTION = 1.5;
//! Usual distance from center point of CROSS_SECTION and traffic sign:
const float TRAFFIC_SIGN_DIST_TO_CROSS_SECTION = 1.031;
//! Maximum allowed distance between traffic sign and center point of STRAIGHT patch:
const float TRAFFIC_SIGN_MAX_DIST_TO_STRAIGHT = 0.9;

//! Signs detected beyond this distance to the car will be ignored (they're often inexact).
const float TRAFFIC_SIGN_DETECTION_RADIUS = 4.5;

class TrafficSign;  // predefine so shared_ptr can use it
typedef boost::shared_ptr<TrafficSign>
    TrafficSignPtr;  // define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<TrafficSign> > TrafficSignPtrList;

class Patch;				    // predefine so shared_ptr can use it
typedef boost::shared_ptr<Patch> PatchPtr;  // define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<Patch> > PatchPtrList;

enum TrafficSignType {
  UNMARKED_INTERSECTION,
  STOP_AND_GIVE_WAY,
  PARKING_AREA,
  HAVE_WAY,
  GIVE_WAY,
  PEDESTRIAN_CROSSING,
  TEST_COURSE_A9,
  ROAD_WORKS,
  INVALID
};

class TrafficSign : public EnvObject {
 public:
  TrafficSign(TrafficSignType type, double x, double y);
  TrafficSign(TrafficSignType type, const oadrive::core::ExtendedPose2d &pose);
  TrafficSign();
  ~TrafficSign();

  //PatchPtr getConnectedPatch() { return mConnectedPatch; }
  //void setConnectedPatch(PatchPtr patch) { mConnectedPatch = patch; }

  TrafficSignType getType() const { return mTrafficSignType; }

  bool isIntersectionTrafficSign() const;

 private:
  //PatchPtr mConnectedPatch;
  TrafficSignType mTrafficSignType;

 public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace world
}  // namespace oadrive

#endif

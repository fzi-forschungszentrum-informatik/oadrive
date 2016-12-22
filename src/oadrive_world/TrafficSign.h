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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-19
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_TRAFFICSIGN_H
#define OADRIVE_WORLD_TRAFFICSIGN_H

#include "EnvObject.h"
#include <oadrive_util/CoordinateConverter.h> 
#include <boost/shared_ptr.hpp>
#include <oadrive_core/ExtendedPose2d.h>
#include "Patch.h"
#include "Obstacle.h"
#include <list>


namespace oadrive{
namespace world{

//! probability added to each detection of the same traffic sign
const float TRAFFIC_SIGN_PROBABILITY_ADD = 0.1;
//! probability must be greater than set value for a traffic sign to connect to a special patch (0.1 means, at least 2 detections)
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
const float TRAFFIC_SIGN_DETECTION_RADIUS = 3.5;

class TrafficSign;	// predefine so shared_ptr can use it
typedef boost::shared_ptr<TrafficSign> TrafficSignPtr;		// define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<TrafficSign> > TrafficSignPtrList;

class Patch;	// predefine so shared_ptr can use it
typedef boost::shared_ptr<Patch> PatchPtr;		// define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<Patch> > PatchPtrList;

class TrafficSign : public EnvObject
{
public:
  TrafficSign( int type, double x, double y );
  TrafficSign( int type, const oadrive::core::ExtendedPose2d &pose );
  ~TrafficSign();

  PatchPtr getConnectedPatch() { return mConnectedPatch; }
  void setConnectedPatch( PatchPtr patch ) { mConnectedPatch = patch; }

  int getSignType() { return mTrafficSignType; }


private:

  PatchPtr mConnectedPatch;
  int mTrafficSignType;
public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}	// namespace
}	// namespace

#endif

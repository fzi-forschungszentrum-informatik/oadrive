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
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_ROADPERCEPTION_MAPPATCHER_H
#define OADRIVE_ROADPERCEPTION_MAPPATCHER_H

#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_world/TrafficSign.h>

#include "oadrive_lanedetection/FeatureDetection/StreetTypes.h"
#include "oadrive_lanedetection/RoadPatching/RoadPatcher.h"

#include "Map.h"


namespace oadrive
{
namespace lanedetection
{


class MapPatcher : public RoadPatcher
{

public:
  MapPatcher(cv::Size2i birdviewSize, util::CoordinateConverter* coordConverter, std::string mapPath);
  PatchHypothesisList generateIntersectionPatches(const OadrivePose worldPose, const PatchHypothesisList& currentPatches);
  void loadMap(std::string map);
  
private:
  void findIntersectionPatch(const OadrivePose worldPose, const PatchHypothesisList& currentPatches);

  Map mMap;

  #ifdef DEBUG_ACTIVE
  virtual void dbgVisualizeVotingSpace() override {
    
  }
  #endif

public:
    // use a proper alignment when calling the constructor.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}    // namespace
}    // namespace




#endif //OADRIVE_ROADPERCEPTION_MAPPATCHER_H

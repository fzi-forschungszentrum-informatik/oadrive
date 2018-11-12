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
 * \author  Fabian Dürr
 * \date    2017-9-26
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_ROADPERCEPTION_INTERSECTIONPATCHER_H
#define OADRIVE_ROADPERCEPTION_INTERSECTIONPATCHER_H

#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_world/TrafficSign.h>

#include "oadrive_lanedetection/FeatureDetection/StreetTypes.h"
#include "oadrive_lanedetection/RoadPatching/RoadPatcher.h"
#include "oadrive_lanedetection/RoadPatching/IntersectionVotingSpace.h"
#include "oadrive_lanedetection/DebugViewRoadPerception.h"


namespace oadrive
{
namespace lanedetection
{


class IntersectionPatcher : public RoadPatcher
{

#ifdef DEBUG_ACTIVE
  friend class DebugViewRoadPerception;
#endif

public:

  IntersectionPatcher(cv::Size2i birdviewSize,
                        util::CoordinateConverter* coordConverter);

  PatchHypothesisList generateIntersectionPatches(const OadrivePose currentCarPose,
                                                  const PatchHypothesisList &currentPatchHyps,
                                                  const PatchHypothesisList &currentCrossPatchHyps,
                                                  const world::TrafficSign
                                                  * currentTrafficSign);

private:

  void findIntersectionPatch();

  PatchHypothesis searchIntersectionInNNSegmentation(const cv::Mat &neuralNetOutput,
                                                     const PatchHypothesisList &currentPatchHyps);

  void addPatchHypothesis(const PatchHypothesis &patchHyp);

  void initPatchVotes();

  void initTrafficSignVotes();

  void addTrafficSign(const world::TrafficSign* trafficSign);

  /*! The voting space the votes of the features are placed in */
  IntersectionVotingSpace mVotingSpace;

#ifdef DEBUG_ACTIVE
  virtual void dbgVisualizeVotingSpace() override;
#endif

public:
    // use a proper alignment when calling the constructor.
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}    // namespace
}    // namespace




#endif //OADRIVE_ROADPERCEPTION_INTERSECTIONPATCHER_H

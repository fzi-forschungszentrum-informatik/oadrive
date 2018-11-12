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

#ifndef OADRIVE_LANEDETECTION_ROADPATCHER_H
#define OADRIVE_LANEDETECTION_ROADPATCHER_H

#include <oadrive_util/CoordinateConverter.h>

#include "oadrive_lanedetection/DebugViewRoadPerception.h"
#include "oadrive_lanedetection/FeatureDetection/StreetTypes.h"
#include "VotingSpace.h"
#include "PatchHypothesis.h"

#define DEBUG_VIEW0(pose) \
        double x0 = pose.getX(); \
        double y0 = pose.getY(); \
        double yaw0 = pose.getYaw(); \

#define DEBUG_VIEW1(pose) \
        double x1 = pose.getX(); \
        double y1 = pose.getY(); \
        double yaw1 = pose.getYaw(); \


namespace oadrive
{
namespace lanedetection
{


/*! \brief Class which should calculate street patches from features which were detected in the bird view image.
 * The class works in car-coordinates, so features which were found on the bird-view should be
 * transformed into car-space (see CoordinateConverter class).
 */
class RoadPatcher
{

#ifdef DEBUG_ACTIVE
  friend class DebugViewRoadPerception;
#endif

protected:

  RoadPatcher(cv::Size2i birdviewSize, util::CoordinateConverter* coordConverter,
              int minVote, int detectionFactor);

  virtual ~RoadPatcher();


  /*! Removes any previously found patches. */
  void clearPatches();

  PatchHypothesis createPatchHypothesis(OadrivePose pose, float score, int id,
                                        PatchHypothesis::Type type);


/******************************************************************************/
/**************************** class attributes ********************************/
/******************************************************************************/

  /*! The list of detected street patches */
  PatchHypothesisList mPatchHypotheses;

  /*! Predefined feature votes for patches: */
  std::vector<std::vector<Vote> > mVotes;

  /*! Predefined patch votes for patches: */
  std::vector<std::vector<Vote> > mPatchHypVotes;

  util::CoordinateConverter* mCoordConverter;

  /*! The minimum vote of a feature */
  int mMinimumVote;

#ifdef DEBUG_ACTIVE
  PatchHypothesis dbgCurrentRefPatch;
  PatchHypothesis dbgNewPatch;
  bool dbgFoundNewPatch = false;
  int dbgCurrentIt;
  bool dbgVisualizationEnabled = false;

  virtual void dbgVisualizeVotingSpace()= 0;
#endif

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}    // namespace
}    // namespace






#endif //OADRIVE_LANEDETECTION_ROADPATCHER_H

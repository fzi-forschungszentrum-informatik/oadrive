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
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_LANEDETECTION_STREETVOTINGSPACE_H
#define OADRIVE_LANEDETECTION_STREETVOTINGSPACE_H

#include "VotingSpace.h"
#include "oadrive_lanedetection/DebugViewRoadPerception.h"

namespace oadrive
{
namespace lanedetection
{


class StreetVotingSpace : public VotingSpace
{

  INIT_MEASURE_EXEC_TIME

#ifdef DEBUG_ACTIVE
  friend class DebugViewRoadPerception;
#endif

public:

  StreetVotingSpace(util::CoordinateConverter* coordConverter, OadrivePose centerPose,
                    cv::Size2i size);

  /*!
  * Add a feature vote to the voting space.
  * @param featurePose The pose of the feature which is responsible for the vote.
  * @param vote The vote which should be placed in the voting space.
  */
  void addFeatureVote(const OadrivePose &featurePose, const Vote &vote);

  /*!
 * Add a patch hypothesis vote to the voting space.
 * @param patchHypPose The beliefed pose of the patch which is responsible for the vote.
 * @param vote The vote which should be placed in the voting space.
 */
  void addPatchHypVote(const OadrivePose &patchHypPose, const Vote &vote);

  /*!
  * Search for maximum in the voting space.
  * @param regionOfInterest determines the area in which the maximum will be searched.
  * @param valueOfMaximum the value of the maximum is saved in here.
  * @return the local poses of the detected maximum.
  */
  virtual OadrivePose searchForMaximum(const RegionOfInterest &regionOfInterest,
                                       int* valueOfMaximum,
                                       bool clear);


  void setThresholdsFeatureOnly(unsigned int minScoreFeatureOnly);
  void setThresholdsCombinedOnly(unsigned int minScoreCombined);
  void setThresholds(unsigned int minScoreCombined, unsigned int minScoreFeatureOnly);


  /*!
    * Clears all votes.
    */
  virtual void clear() override;

private:

  VotingSpace::MaxVote extractMaximum(RegionOfInterest roi, int refPatchAngleRegion,
                                      bool useCombined);

  void setRegion(int angleRegion, cv::Point p, cv::Size size, int newAmount);

private:

  /*! The voting space. Three dimensions for x,y and angle. */
  VotingContainer mHoughSpaceFeatures;
  VotingContainer mHoughSpacePatches;

  unsigned int mMinScoreCombined;
  unsigned int mMinScoreFeatureOnly;
  bool mFeatureOnlySearch;
  bool mCombinedSearch;

};

}

}

#endif // OADRIVE_LANEDETECTION_STREETVOTINGSPACE_H


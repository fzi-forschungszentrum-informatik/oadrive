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

#ifndef OADRIVE_LANEDETECTION_INTERSECTIONVOTINGSPACE_H
#define OADRIVE_LANEDETECTION_INTERSECTIONVOTINGSPACE_H

#include "VotingSpace.h"
#include "oadrive_lanedetection/DebugViewRoadPerception.h"

namespace oadrive
{
namespace lanedetection
{


class IntersectionVotingSpace : public VotingSpace
{

#ifdef DEBUG_ACTIVE
  friend class DebugViewRoadPerception;
#endif

public:

  IntersectionVotingSpace(util::CoordinateConverter* coordConverter,
                            cv::Size2i size);

  void addCrossPatchVote(const OadrivePose &featurePose, const Vote &vote);

  void addPatchVote(const OadrivePose &patchPose, const Vote &vote);

  void addTrafficSignVote(const OadrivePose &trafficSignLocalPose, const Vote &vote);


  /*!
 * Search for maxima in the voting space.
 * @param minimumVoteForDetection the minimum value requiered to consider a location a maximum.
 * @param numOfMax the number of maxima to find.
 * @param valueOfMaximum the values of the detected maxima are saved in this list.
 * @return a list with the global poses of the detected maxima
 */
  virtual OadrivePose searchForMaximum(int* valueOfMaximum, bool* basedOnTafficSign);


  /*!
    * Clears all votes.
    */
  virtual void clear() override;

  /*!
   *
   */
  void clearPatchHypVotes();

private:

  VotingSpace::MaxVote extractMaximum(const VotingContainer &votingSpaceA,
                                      const VotingContainer &votingSpaceB);


private:

  /*! The voting space. Three dimensions for x,y and angle. */
  //std::vector<cv::Mat> mHoughSpaceFeatures;

  VotingContainer mHoughSpacePatches;
  VotingContainer mHoughSpaceCrossPatches;
  VotingContainer mHoughTrafficSigns;
};

}

}

#endif // OADRIVE_LANEDETECTION_STREETVOTINGSPACE_H


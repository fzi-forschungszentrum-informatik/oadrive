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

#include "RoadPatcher.h"
#include "oadrive_lanedetection/lanedetectionLogging.h"


using oadrive::core::Position2d;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace lanedetection
{

RoadPatcher::RoadPatcher(cv::Size2i birdviewSize, util::CoordinateConverter* coordConverter,
                         int minVote, int detectionFactor) :
        mPatchHypotheses(0),
        mVotes(numFeatureTypes, std::vector<Vote>()),
        mPatchHypVotes(PatchHypothesis::numPatchTypes, std::vector<Vote>()),
        mCoordConverter(coordConverter),
        mMinimumVote(10)
{
}

RoadPatcher::~RoadPatcher()
{
}


/******************************************************************************/
/*************************** protected methods ********************************/
/******************************************************************************/

PatchHypothesis
RoadPatcher::createPatchHypothesis(OadrivePose pose, float score, int id,
                                   PatchHypothesis::Type type)
{
  float probability = std::min(float(score) / float(MAX_VOTING_SCORE), 1.f);
  PatchHypothesis newPatch(pose, id, probability, type, PatchHypothesis::BasedOn::FEATURE);

  return newPatch;
}

void RoadPatcher::clearPatches()
{
  mPatchHypotheses.clear();
}

}   // namespace
}   // namespace


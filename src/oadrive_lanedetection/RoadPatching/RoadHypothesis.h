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

#ifndef OADRIVE_LANEDETECTION_ROADHYPOTHESIS_H
#define OADRIVE_LANEDETECTION_ROADHYPOTHESIS_H

#include "PatchHypothesis.h"

#include "oadrive_util/CoordinateConverter.h"

namespace oadrive
{
namespace lanedetection
{

class RoadHypothesis
{

public:

  RoadHypothesis(util::CoordinateConverter* coordConverter);

  /*! Determines the new reference patch and removes it from the current patch list. Other patches
   * not visible in the birdview anymore will be also removed. After this only patches visible in
   * the birdview will be in the current patch list.
   */
  void updatePatches(OadrivePose currentCarPose, OadrivePose previousCarPose);

  /*! Adds the patches found in the current timestep. If a patch with the same ID already exists,
   * the two patches will be merged, because they can be viewed as two hypothesis for the same
   * patch. Otherwise the found patch will be added to the list of current patches.
   * @param foundPatches the patches found in this timestep.
   */
  void addPatches(PatchHypothesisList &foundPatches);

  void addIntersection(PatchHypothesis &foundIntersection,
                       const PatchHypothesisList &currentCrossPatches);

  void setInitialPatchPose(const OadrivePose patchPose);

  PatchHypothesisList &getRoadHypothesis();
  PatchHypothesisList getSortedRoadHypothesis();

  PatchHypothesis &getCurrentRefHyp();
  PatchHypothesis *getCurrentIntersection();

  const std::list<float> &getAngleHistory() const;

  void addNextDrivingDirection(PatchHypothesis::DrivingDirection driveDir);

  void reset(const OadrivePose &estimatedPatchPose);

private:

  bool updateExistingIntersection(PatchHypothesis &newIntersection);

  int checkForPatchRemoval(const PatchHypothesis &intersection);

  PatchHypothesis getNearest(PatchHypothesisList patchHyps, PatchHypothesis intersection);

  void addAngleToHistory(const PatchHypothesis &patchHyp);

  /******************************************************************************/
/**************************** class attributes ********************************/
/******************************************************************************/

  // The patches currently visible in the birdview. As a result they could still change.
  PatchHypothesisList mPatchHypotheses;

  // The current reference patch hypothesis, which is the first patch (from the end) not visible in
  // the birdview anymore .
  PatchHypothesis mReferenceHypothesis;

  util::CoordinateConverter* mCoordConverter;

  // Array with the angle differences between the last 6 patches.
  std::list<float> mAngleHistory;

  PatchHypothesis::DrivingDirection mCurrentDrivingCommand;

  // The estimated pose of the first patch which is used as start for the patch search.
  // oadrivePose mEstimatedInitalPatchPose;

};

}
}

#endif //OADRIVE_LANEDETECTION_ROADHYPOTHESIS_H

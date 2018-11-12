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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-24
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "Road.h"
#include "oadrive_world/Patch.h"
#include "worldLogging.h"

#include <oadrive_lanedetection/lanedetectionLogging.h>
#include <boost/make_shared.hpp>

using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace oadrive::core;

// define PI/8 since it isn't implemented in C++ by default
#define M_PI_8 0.3926990816987241548078304229099378605246461749218882

namespace oadrive
{
namespace world
{

Road::Road() :
        mPatches(0),
        mOpenPatches(0),
        mMaximumAngle(M_PI),
        mMinimumDistBetweenCrossings(1.7)
{
}

/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/

void Road::addRoadPatch(PatchPtr streetPatch)
{
  bool replaced = false;
  for (auto it = mPatches.begin(); it != mPatches.end(); it++)
  {
    if ((*it)->getPatchID() == streetPatch->getPatchID())
    {
      // Merge new data from update into the patch
      (*it)->mergeFrom(streetPatch);

      replaced = true;
      break;
    }
  }

  if(!replaced)
  {
    // Set the patch to the current lane:
    streetPatch->setLane(mCurrentLane);
    mPatches.push_back(streetPatch);
  }

  for (auto it = mPatches.begin(); it != mPatches.end(); it++)
  {
    (*it)->removeSuccessor();

    auto successor = it;
    successor++;

    if (successor != mPatches.end())
    {
      (*it)->setSuccessor(*successor);

      // Move the successor to the center lane, if we have a recommendation for that lane
      if ((*it)->getRecommendedLane() == LANE_CENTER && (*successor)->getLane() == LANE_RIGHT) {
        (*successor)->setLane(LANE_CENTER);
      }
    }
  }
}

std::vector<int> Road::updateHistory(const ExtendedPose2d &currentCarPose)
{
  PatchPtrList newCarPatches;
  std::vector<int> newCarPatchesIDs;

  //get current patches the car is on
  for (PatchPtrList::iterator it = mPatches.begin(); it != mPatches.end(); it++)
  {
    if ((*it)->isPointInside(currentCarPose))
    {
      newCarPatches.push_back((*it));
    }
  }

  //add new patch to the patch history
  bool isNew = true;
  for (PatchPtrList::iterator newPatchIt = newCarPatches.begin();
       newPatchIt != newCarPatches.end(); newPatchIt++)
  {
    isNew = 1;
    for (PatchPtrList::iterator currentPatchIt = mCurrentCarPatches.begin();
         currentPatchIt != mCurrentCarPatches.end(); currentPatchIt++)
    {
      if ((*newPatchIt)->getPatchID() == (*currentPatchIt)->getPatchID())
      {
        isNew = false;
      }
    }

    //add new patch to the past patches the car was on
    if (isNew)
    {
      mCarPatchHistory.push_back((*newPatchIt));
      newCarPatchesIDs.push_back((*newPatchIt)->getPatchID());
    }
  }

  if (newCarPatches.size() > 0)
  {
    mCurrentCarPatches = newCarPatches;
  }
  return newCarPatchesIDs;
}

const PatchPtrList* Road::getCarPatchHistory() const
{
  return &mCarPatchHistory;
}

const PatchPtrList* Road::getRoad() const
{
  return &mPatches;
}

void Road::addTrafficSign(const TrafficSignPtr trafficSign)
{
  if (!trafficSign->isIntersectionTrafficSign()) return;
  
  PatchPtr nearestCrossing;
  float minDist = 2.0;
  bool found = false;
  for (auto patchHypIt = mPatches.begin(); patchHypIt != mPatches.end(); patchHypIt++)
  {
    if ((*patchHypIt)->getPatchType() != CROSS_SECTION)
    {
     continue;
    }


    float distance = (*patchHypIt)->calcDistTo(trafficSign->getPose());
   // LOGGING_WARNING(worldLogger, "Dist: " << distance << endl);
    if (distance < minDist)
    {
      minDist = distance;
      nearestCrossing = (*patchHypIt);
      found = true;
    }
  }

  if (found)
  {
    // check angle
    // float diffAngle = (nearestCrossing->getYaw()) - (trafficSign->getYaw() + M_PI);
    // if (std::abs(diffAngle) < M_PI_4)

    // If the intersection is already connected with a traffic sign, first check if the old sign is closer
    if (nearestCrossing->hasTrafficSign()) {
      auto otherSign = nearestCrossing->getTrafficSign();

      const float otherDist = nearestCrossing->calcDistTo(otherSign->getPose());

      if (otherDist < minDist) {
        // We keep the currently connected sign
        return;
      }
    }
    
    nearestCrossing->connectTrafficSign(trafficSign);
  }
}

void Road::deletePatchesUntilLastIntersection()
{
  PatchPtr patch = mCurrentCarPatches.front();
  auto currentPatch = std::find(mPatches.begin(), mPatches.end(), patch);
  currentPatch++;

  PatchPtrList::iterator lastIntersection;
  bool found = false;
  for (auto patchIt = mPatches.begin(); patchIt != currentPatch; patchIt++)
  {
   if ((*patchIt)->getPatchType() == CROSS_SECTION)
   {
     lastIntersection = patchIt;
     found = true;
   }
  }

  if (found)
  {
    for (auto deletePatchIt = mPatches.begin(); deletePatchIt != lastIntersection;)
    {
      deletePatchIt = mPatches.erase(deletePatchIt);
    }
  }
}

const boost::shared_ptr<Patch> Road::getNextIntersection()
{
  // PatchPtr patch = mCurrentCarPatches.front();
  // auto patchIt = std::find(mPatches.begin(), mPatches.end(), patch);

  for (auto patchIt = mPatches.begin(); patchIt != mPatches.end(); patchIt++)
  {
    if ((*patchIt)->getPatchType() == CROSS_SECTION)
    {
      return (*patchIt);
    }
  }

  std::cout << "I havent found anything :( Num Patches: " << mPatches.size() << std::endl;

  // return nullptr, if no intersection was found
  return PatchPtr();
}

void Road::reset()
{
  mPatches.clear();
  mOpenPatches.clear();
  mCurrentCarPatches.clear();
  mCarPatchHistory.clear();
}

void Road::deleteOldPatches()
{
  auto deletePatchIt = mPatches.begin();

  // Only keep 7 newest patches
  int remove = mPatches.size() - 7;
  for (int i = 0; i < remove; i++)
  {
    // if ((*deletePatchIt)->getPatchType() == CROSS_SECTION) {
    //   deletePatchIt++;
    //   continue;
    // }
    deletePatchIt = mPatches.erase(deletePatchIt);
  }
}

void Road::setLane(LaneType lane, int numChangedOldPatches) {
  mCurrentLane = lane;

  auto patchIt = mPatches.begin();

  for (;patchIt != mPatches.end(); patchIt++) {
    (*patchIt)->setLane(lane);
  }

  patchIt = mCarPatchHistory.begin();

  std::advance(patchIt, mCarPatchHistory.size() - numChangedOldPatches - 1);

  for (;patchIt != mCarPatchHistory.end(); patchIt++) {
    (*patchIt)->setLane(lane);
  }
}

/******************************************************************************/
/**************************** private methods *********************************/
/******************************************************************************/

}  // namespace
}  // namespace

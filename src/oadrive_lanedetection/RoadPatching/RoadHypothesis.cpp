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

#include "oadrive_lanedetection/lanedetectionLogging.h"
#include "RoadHypothesis.h"

using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace lanedetection
{

RoadHypothesis::RoadHypothesis(util::CoordinateConverter* coordConverter) :
        mPatchHypotheses(0),
        mCoordConverter(coordConverter)
{

}

/******************************************************************************/
/****************************** public methods ********************************/
/******************************************************************************/

void RoadHypothesis::updatePatches(OadrivePose currentCarPose, OadrivePose previousCarPose)
{
  bool carIsMoving = (currentCarPose.distance(previousCarPose) > 0.001);

  // Update the local position of the reference hypothesis.
  if (carIsMoving)
  {
    OadrivePose globalPose =
            mCoordConverter->car2World(previousCarPose, mReferenceHypothesis.getPose());
    OadrivePose updatedLocalPose = mCoordConverter->world2Car(currentCarPose, globalPose);
    mReferenceHypothesis.setPose(updatedLocalPose);
  }

  // Update the local positions of the other patch hypothesis.
  int highestDeletedId = -1;
  for (auto it = mPatchHypotheses.begin(); it != mPatchHypotheses.end();)
  {
    OadrivePose globalPose = mCoordConverter->car2World(previousCarPose, (*it).getPose());
    OadrivePose updatedLocalPose = mCoordConverter->world2Car(currentCarPose, globalPose);
    (*it).setPose(updatedLocalPose);


    float distance = currentCarPose.distance(globalPose);
    if (distance < mCoordConverter->getBirdviewOffsetX())
    {
      highestDeletedId = std::max(highestDeletedId, static_cast<int>((*it).getPatchID()));

      mReferenceHypothesis = (*it);
      addAngleToHistory((*it));
      it = mPatchHypotheses.erase(it);
    }
    else
    {
      it++;
    }
  }

  // Delete all patches whose parent got deleted
  if (highestDeletedId >= 0) {
    for (auto it = mPatchHypotheses.begin(); it != mPatchHypotheses.end();) {
      if ((*it).getPatchID() < highestDeletedId) {
        it = mPatchHypotheses.erase(it);
      } else {
        it++;
      }
    }
  }
}

void RoadHypothesis::addPatches(PatchHypothesisList &foundPatches)
{
  PatchHypothesisList patchesToAdd;
  for (auto newPatchHyp = foundPatches.begin(); newPatchHyp != foundPatches.end(); newPatchHyp++)
  {
    bool merged = false;
    for (auto currentPatchHyp = mPatchHypotheses.begin();
         currentPatchHyp != mPatchHypotheses.end(); currentPatchHyp++)
    {
      if ((*currentPatchHyp).getPatchID() == (*newPatchHyp).getPatchID())
      {
        assert((*currentPatchHyp).getPatchType() == (*newPatchHyp).getPatchType());
        assert((*newPatchHyp).getPatchType() != PatchHypothesis::Type::INTERSECTION);

        (*currentPatchHyp).discountProbability(DISCOUNTFACTOR);
        (*currentPatchHyp).mergeHypothesis((*newPatchHyp));
        merged = true;
        break;
      }
    }

    if (!merged)
    {
      patchesToAdd.push_back((*newPatchHyp));
    }
  }

  for (auto patchToAdd = patchesToAdd.begin(); patchToAdd != patchesToAdd.end(); patchToAdd++)
  {
    mPatchHypotheses.push_back(*patchToAdd);
  }
}

void RoadHypothesis::addIntersection(PatchHypothesis &foundIntersection,
                                     const PatchHypothesisList &currentCrossPatches)
{
  // The initial ID is the next free ID. It will be changed if other patches are removed.
  foundIntersection.setPatchID(mPatchHypotheses.back().getPatchID() + 1);
  LOGGING_INFO(lanedetectionLogger, "size: " << currentCrossPatches.size() << endl);
  if (currentCrossPatches.size() > 0 &&
      !(foundIntersection.getBasedOn() == PatchHypothesis::BasedOn::TRAFFIC_SIGN))
  {
    const PatchHypothesis nearestCross = getNearest(currentCrossPatches, foundIntersection);
    const PatchHypothesis nearestRoad = getNearest(mPatchHypotheses, foundIntersection);

    float crossAngle = nearestCross.getYaw() > 0.f ? nearestCross.getYaw() - M_PI_2
                                                   : nearestCross.getYaw() + M_PI_2;

    float resultingAngle = (crossAngle + nearestRoad.getYaw() * 2) / 3.f;
    foundIntersection.setYaw(resultingAngle);
  }

  // Refine map intersections with next and previous patch, this is very hacky right now 
  // Disabled as it does not work very well
  if (false && foundIntersection.getBasedOn() == PatchHypothesis::BasedOn::MAP) {
    // First find next and prev patch
    const PatchHypothesis* previousPatch = nullptr;
    bool prevFound = false;
    const PatchHypothesis* nextPatch = nullptr;
    for (const PatchHypothesis &patch : mPatchHypotheses) {
      if (patch.getPatchType() != PatchHypothesis::Type::ROAD ) {
        // Check if this is "our" intersection
        // if (std::sqrt(std::pow(foundIntersection.getX() - patch.getX(), 2) + std::pow(foundIntersection.getX() - patch.getX(), 2)
        if (patch.getPose().distance(foundIntersection.getPose()) < 0.5) {
          prevFound = true;
        }
      } else {
        if (!prevFound) {
          previousPatch = &patch;
        } else {
          nextPatch = &patch;
          break;
        }
      }
    }

    if (previousPatch && nextPatch && nextPatch->getBasedOn() == PatchHypothesis::BasedOn::FEATURE) {
      std::cout << "prev " << previousPatch->getPatchID() << " next: " << nextPatch->getPatchID() << std::endl;

      if (mCurrentDrivingCommand == PatchHypothesis::DrivingDirection::STRAIGHT) {
        // If we go straight we can simply average the coordinates of both patches to get the intersection
        foundIntersection.setX((previousPatch->getX() + nextPatch->getX()) / 2);
        foundIntersection.setY((previousPatch->getY() + nextPatch->getY()) / 2);
        
        // weighted avg yaw 1 * prev + 3 * next
        foundIntersection.setYaw(atan2(sin(previousPatch->getYaw()) + 3 * sin(nextPatch->getYaw()), cos(previousPatch->getYaw()) + 3 * cos(nextPatch->getYaw())));
      } else {
        const float prevX = previousPatch->getX();
        const float prevY = previousPatch->getY();
        const float prevYaw = previousPatch->getYaw();
        const float normalVecPrevX = 0.5 * cos(prevYaw);
        const float normalVecPrevY = 0.5 * sin(prevYaw);

        const float nextX = nextPatch->getX();
        const float nextY = nextPatch->getY();
        const float nextYaw = nextPatch->getYaw();
        const float normalVecNextX = 0.5 * cos(nextYaw);
        const float normalVecNextY = 0.5 * sin(nextYaw);

        float t3 = nextY * normalVecPrevX / normalVecPrevY - prevY * normalVecPrevX / normalVecPrevY - nextX + prevX;
        t3 /= (normalVecNextX - normalVecNextY * normalVecPrevX / normalVecPrevY);
        
        float interX = nextX + t3 * normalVecNextX;
        float interY = nextY + t3 * normalVecNextY;


        float yawFromNext = nextPatch->getYaw();

        if (mCurrentDrivingCommand == PatchHypothesis::DrivingDirection::LEFT) {
          yawFromNext -= M_PI_2;
        } else if (mCurrentDrivingCommand == PatchHypothesis::DrivingDirection::LEFT) {
          yawFromNext += M_PI_2;
        }

        foundIntersection.setX(interX);
        foundIntersection.setY(interY);
        // weighted avg yaw 1 * prev + 3 * next
        foundIntersection.setYaw(atan2(sin(previousPatch->getYaw()) + 3 * sin(yawFromNext), cos(previousPatch->getYaw()) + 3 * cos(yawFromNext)));
      }

      /*
      

*/
    } else {
      // std::cout << previousPatch <<  " " << nextPatch << std::endl;
    }
  }



  // check if we already have an intersectionHyp. Than we have to merge or discard the new one.
  bool merged = updateExistingIntersection(foundIntersection);

  if (merged)
  {
    for (auto patchHypIt = mPatchHypotheses.begin();
         patchHypIt != mPatchHypotheses.end(); patchHypIt++)
    {
      if ((*patchHypIt).getPatchType() == PatchHypothesis::Type::INTERSECTION)
      {
        checkForPatchRemoval((*patchHypIt));
        break;
      }
    }
  }
  else
  {
    // check if normal road patches have to be removed because they are located inside the
    // updated/new intersection
    int intersectionID = checkForPatchRemoval(foundIntersection);
    foundIntersection.setPatchID(intersectionID);

    foundIntersection.setDrivingDirection(mCurrentDrivingCommand);

    mPatchHypotheses.push_back(foundIntersection);
  }
}

void RoadHypothesis::setInitialPatchPose(const OadrivePose patchPose)
{
  mReferenceHypothesis = PatchHypothesis(patchPose, 0, 0.f,
                                         PatchHypothesis::Type::ROAD,
                                         PatchHypothesis::BasedOn::PREDICTION);

  if (mPatchHypotheses.size() > 0) {
    mReferenceHypothesis.setPatchID(mPatchHypotheses.back().getPatchID() + 1);
  }
}

PatchHypothesisList &RoadHypothesis::getRoadHypothesis()
{
  return mPatchHypotheses;
}

PatchHypothesisList RoadHypothesis::getSortedRoadHypothesis()
{
  PatchHypothesisList sortedList = mPatchHypotheses;

  sortedList.sort([](const PatchHypothesis & a, const PatchHypothesis & b) -> bool
    { 
      return a.getPatchID() < b.getPatchID();
    }
  );

  return sortedList;
}

PatchHypothesis &RoadHypothesis::getCurrentRefHyp()
{
  return mReferenceHypothesis;
}

PatchHypothesis *RoadHypothesis::getCurrentIntersection() {
  for (auto patchHypIt = mPatchHypotheses.begin(); patchHypIt != mPatchHypotheses.end(); patchHypIt++)
  {
    if ((*patchHypIt).getPatchType() == PatchHypothesis::Type::INTERSECTION)
    {
      return &(*patchHypIt);
    }
  }

  return nullptr;
}

const std::list<float> &RoadHypothesis::getAngleHistory() const
{
  return mAngleHistory;
}

/******************************************************************************/
/***************************** private methods ********************************/
/******************************************************************************/

bool RoadHypothesis::updateExistingIntersection(PatchHypothesis &newIntersection)
{
  float maxDistForMerge = 1.25f;

  for (auto patchHypIt = mPatchHypotheses.begin();
       patchHypIt != mPatchHypotheses.end(); patchHypIt++)
  {
    if ((*patchHypIt).getPatchType() == PatchHypothesis::Type::INTERSECTION)
    {
      // If another intersection already exist, check the distance between it and the new one.
      // If they are close enough merge them otherwise dump the new one.
      float dist = (*patchHypIt).getPose().distance(newIntersection.getPose());

      if (dist < maxDistForMerge)
      {
        // merged
        (*patchHypIt).discountProbability(DISCOUNTFACTOR);
        (*patchHypIt).mergeHypothesis(newIntersection);
        return true;
      }
      else
      {
        // dump because to far away. But return true so the dumped intersection is not used anymore
        // by the calling functions.
        return true;
      }
    }
  }
  // not merged and dumped because we have no other intersection.
  return false;
}

int RoadHypothesis::checkForPatchRemoval(const PatchHypothesis &intersection)
{
  int intersectionID = intersection.getPatchID();
  for (auto patchHypIt = mPatchHypotheses.begin(); patchHypIt != mPatchHypotheses.end();)
  {
    OadrivePose poseRelPatch =
            mCoordConverter->world2Car(intersection.getPose(), (*patchHypIt).getPose());

    // Search for road patches located inside the intersection and delete it.
    // The ID for the intersection is the smallest id of the removed road patches.
    if ((*patchHypIt).getPatchType() == PatchHypothesis::Type::ROAD)
    {
      // is inside the intersection
      if (std::abs(poseRelPatch.getY()) < PATCH_WIDTH_INTERSECTION * 0.5f &&
          std::abs(poseRelPatch.getX()) < PATCH_LENGTH_INTERSECTION * 0.5f)
      {

        // Store smallest id from removed patches
        if ((*patchHypIt).getPatchID() < intersectionID)
        {
          intersectionID = (*patchHypIt).getPatchID();
        }

        patchHypIt = mPatchHypotheses.erase(patchHypIt);
        continue;
      }
    }

    patchHypIt++;
  }

  return intersectionID;
}

void RoadHypothesis::addAngleToHistory(const PatchHypothesis &patchHyp)
{
  // If the patch to add is a intersection clear the history because it doesn't make sense
  // to calculate a mean out of patches before and after a intersection where we are turning.
  // Because the orientation of intersection aren't that robust we clear the history in any case.
  if (patchHyp.getPatchType() == PatchHypothesis::Type::INTERSECTION)
  {
    mAngleHistory.clear();
  }

  mAngleHistory.push_front(patchHyp.getYaw());

  if (mAngleHistory.size() > 6)
  {
    mAngleHistory.pop_back();
  }
}

PatchHypothesis
RoadHypothesis::getNearest(PatchHypothesisList patchHyps, PatchHypothesis intersection)
{
  PatchHypothesis nearest;
  float minY = std::numeric_limits<float>().max();

  for (const PatchHypothesis &patchHyp : patchHyps)
  {
    OadrivePose poseRelPatch = mCoordConverter->world2Car(intersection.getPose(),
                                                          patchHyp.getPose());

    if (std::abs(poseRelPatch.getY()) < minY &&
        (patchHyp.getPatchType() == PatchHypothesis::Type::ROAD ||
         patchHyp.getPatchType() == PatchHypothesis::Type::CROSSING_ROAD))
    {
      minY = std::abs(poseRelPatch.getY());
      nearest = patchHyp;
    }
  }

  return nearest;
}

void RoadHypothesis::addNextDrivingDirection(PatchHypothesis::DrivingDirection driveDir)
{
  mCurrentDrivingCommand = driveDir;
}


void RoadHypothesis::reset(const OadrivePose &estimatedPatchPose)
{
  setInitialPatchPose(estimatedPatchPose);
  mPatchHypotheses.clear();
  mAngleHistory.clear();
}

}
}
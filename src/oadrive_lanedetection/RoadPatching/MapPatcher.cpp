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

#include "MapPatcher.h"
#include <oadrive_lanedetection/lanedetectionLogging.h>
#define TIXML_USE_TICPP
#include <tinyxml.h>

using oadrive::util::ImageHelper;
using oadrive::world::TrafficSign;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace lanedetection
{

MapPatcher::MapPatcher(cv::Size2i birdviewSize,
                                         util::CoordinateConverter* coordConverter, std::string mapPath) :
        RoadPatcher(birdviewSize, coordConverter, 10, 5),
        mMap(mapPath)
{
  LOGGING_INFO(lanedetectionLogger, "Generating MapPatcher from " << mapPath << endl);
}

/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/

PatchHypothesisList
MapPatcher::generateIntersectionPatches(const OadrivePose worldPose, const PatchHypothesisList& currentPatches)
{
  // Remove old features
  clearPatches();

  // search for new ones:
  findIntersectionPatch(worldPose, currentPatches);

  return mPatchHypotheses;
}

/******************************************************************************/
/***************************** private methods ********************************/
/******************************************************************************/

void MapPatcher::findIntersectionPatch(const OadrivePose worldPose, const PatchHypothesisList& currentPatches)
{
  // OadrivePose worldPatchPose(1.42527, -1.85314, M_PI / 2 + 1.63752);
  // mMap.queryIntersections(worldPatchPose, 1.0);

  // return;
  // Straight patches can introduce an intersection
  std::vector<Map::Tile> lastIntersections;
  const PatchHypothesis* previousPatch = nullptr;
  // This construction prevents multiple patches detecting the same intersection
  for (const PatchHypothesis &patch : currentPatches) {
    if (patch.getPatchType() != PatchHypothesis::Type::ROAD) {
      break;
    }

    OadrivePose worldPatchPose = mCoordConverter->car2World(worldPose, patch.getPose());
    // std::cout << "car " << worldPose.getX() << " " << worldPose.getY() << " " << worldPose.getYaw() << " patch " << worldPatchPose.getX() << " " << worldPatchPose.getY() << " " << worldPatchPose.getYaw() << std::endl;

    auto intersections = mMap.queryIntersections(worldPatchPose, 0.8);

    if (intersections.size() > 0) {
      lastIntersections = intersections;
      previousPatch = &patch;
      
      // std::cout << "car " << worldPose.getX() << " " << worldPose.getY() << " " << worldPose.getYaw() << " patch " << worldPatchPose.getX() << " " << worldPatchPose.getY() << " " << worldPatchPose.getYaw() << std::endl;
    
    }
  }
  
  if (lastIntersections.size() > 0) {
    for (auto intersection : lastIntersections) {
      float x = intersection.pose.getX() - worldPose.getX();
      float y = intersection.pose.getY() - worldPose.getY();
      float yaw = -worldPose.getYaw();

      float predictedX = x * cos(yaw) - y * sin(yaw);
      float predictedY = x * sin(yaw) + y * cos(yaw);

      float prevX = previousPatch->getX();
      float prevY = previousPatch->getY();
      float prevYaw = previousPatch->getYaw();

      /*
      Intersection (Schnitt!) between two lines to determine intersection position
      if there is no next yet use straight line with predictedX
      g1 = (predictedX, 0) + t1*(0,1)
      g2 = (prevX, prevY) + t2 * normalVecPrev
      g1 == g2: (predictedX, 0) + t1*(0,1) = (prevX, prevY) + t2 * normalVecPrev
      predictedX = prevX + t2 * normVecX => t2 = (predictedX - prevX) / normVecX
      t1 = prevY + t2 * normVecY => 
      */

      float normalVecPrevX = 0.5 * cos(prevYaw);
      float normalVecPrevY = 0.5 * sin(prevYaw);
      // std::cout << "prev norm vec " << normalVecPrevX << " " << normalVecPrevY << std::endl;

      // Code if there would only be one patch to make an intersection
      float t2 = (predictedX - prevX) / normalVecPrevX;
      float t1 = prevY + t2 * normalVecPrevY;

      // => Intersection calculated with g1: (predictedX, predictedY) = (predictedX, t1 * 1)
      // predictedY = t1;

      // Find rotation in PI/2

      float prevPatchYawInWorld = fmod(M_PI + prevYaw + worldPose.getYaw(), 2 * M_PI); // Patch rot in world coordinates and between 0, 2Pi
      float minVal = 10000;
      float minYaw = 0;

      // float lokalesMPI2 = M_PI_2;
      for (int i = 0; i < 4; i++) {
        const float x = i * M_PI_2;
        float dist = fmin((2 * M_PI) - fabs(x - prevPatchYawInWorld), fabs(x - prevPatchYawInWorld));
        // float dist = fabs(i * lokalesMPI2 - prevPatchYawInWorld);
        if (dist < minVal) {
          minVal = dist;
          minYaw = x - worldPose.getYaw() - M_PI; // Back into car coordinates -Pi, Pi
        }
      }

      // PatchHypothesis newPatch = createPatchHypothesis(OadrivePose(predictedX, predictedY, prevYaw), 1, -1, PatchHypothesis::Type::INTERSECTION);
      PatchHypothesis newPatch = createPatchHypothesis(OadrivePose(predictedX, predictedY, minYaw), 1, -1, PatchHypothesis::Type::INTERSECTION);
      newPatch.setBasedOn(PatchHypothesis::BasedOn::MAP);

      mPatchHypotheses.push_back(newPatch);
      break;
    }
  }


  return;
}

void MapPatcher::loadMap(std::string map) {
  mMap.loadFromString(map);
}

}
}
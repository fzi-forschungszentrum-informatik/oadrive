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

#ifndef OADRIVE_LANEDETECTION_PATCHHYPOTHESIS_H
#define OADRIVE_LANEDETECTION_PATCHHYPOTHESIS_H

#include "oadrive_lanedetection/lanedetectionLogging.h"
#include "oadrive_lanedetection/FeatureDetection/StreetTypes.h"
#include "RoadPerceptionConfig.h"


namespace oadrive
{
namespace lanedetection
{

  #ifdef DEBUG_ACTIVE
    class DebugViewRoadPerception;
  #endif

struct RegionOfInterest
{
  // center of the region of interest in world coordinates
  OadrivePose center;

  // the size in x-direction
  float length;

  // the size in y-direction
  float width;
};


class PatchHypothesis;
typedef std::list<PatchHypothesis> PatchHypothesisList;

class PatchHypothesis
{

#ifdef DEBUG_ACTIVE
  friend class DebugViewRoadPerception;
#endif

public:

  enum Type
  {
    ROAD, CROSSING_ROAD, INTERSECTION, PARKING, TRAFFICSIGN
  };
  static const int numPatchTypes = 5;


  enum BasedOn
  {
    FEATURE, PREVIOUS_HYP, PREDICTION, TRAFFIC_SIGN, MAP
  };

  enum DrivingDirection
  {
    STRAIGHT, LEFT, RIGHT
  };

public:

  PatchHypothesis(OadrivePose pose, uint ID, float probability, Type type, BasedOn based);

  PatchHypothesis() {}

  void mergeHypothesis(const PatchHypothesis &other);

  RegionOfInterest getROIForNextPatch() const;

  std::vector<RegionOfInterest> getROIForCrossPatches() const;

  std::vector<RegionOfInterest> getROIsForIntersection(float angleRegionSize) const;

  RegionOfInterest getROIForParkingSpot() const;

  int getPatchID() const;
  void setPatchID(int id);

  OadrivePose getPose() const;
  void setPose(OadrivePose pose);

  float getX() const;
  void setX(float x);

  float getY() const;
  void setY(float y);

  float getYaw() const;
  void setYaw(float y);

  float getProbability() const;
  void discountProbability(float factor);

  Type getPatchType() const;

  BasedOn getBasedOn() const;
  void setBasedOn(BasedOn basedOn);

  DrivingDirection getDrivingDirection() const;
  void setDrivingDirection(DrivingDirection drivingDirection);

  cv::Size2f getPatchSize() const;

private:

  OadrivePose calculatePoseWithOffset(float offsetX, float offsetY, float rotateOffset,
                                      float offsetYaw) const;


  // The pose of the patch
  OadrivePose mPose;

  // ID of patch the hypothesis belongs to. Consecutive patches have consecutive IDs.
  int mPatchID;

  // The probability of the hypothesis to be true
  float mProbability;

  // The type of the patch: STRAIGHT, CROSSSECTION, PARKING
  Type mPatchType;

  // The method how the hypothesis was created Based on found haar features or predicted by
  // the patch before.
  BasedOn mBasedOn;

  DrivingDirection mDrivingDirection;

  static float ROI_LENGTH;
  static float ROI_WIDTH;

};

std::ostream& operator << (std::ostream &os, const PatchHypothesis &patchHyp);
icl_core::logging::ThreadStream& operator << (icl_core::logging::ThreadStream &os,
                                              const PatchHypothesis &patchHyp);


}
}

#endif //OADRIVE_LANEDETECTION_PATCHHYPOTHESIS_H

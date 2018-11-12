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

#ifndef PROJECT_DEBUGVIEWROADPERCPTION_H
#define PROJECT_DEBUGVIEWROADPERCPTION_H

#include <oadrive_util/CoordinateConverter.h>
#include <opencv2/core/mat.hpp>

#include "oadrive_lanedetection/FeatureDetection/HaarFeature.h"
#include "oadrive_lanedetection/RoadPatching/PatchHypothesis.h"


/*********************************************************************/
/*********************** Time Measurement ****************************/
/*********************************************************************/
#define MEASURE_EXEC_TIME

#ifdef MEASURE_EXEC_TIME
#define INIT_MEASURE_EXEC_TIME std::chrono::high_resolution_clock::time_point t1, t2;\
                               float duration;

#else
#define INIT_MEASURE_EXEC_TIME
#endif

#ifdef MEASURE_EXEC_TIME
#define START_CLOCK t1 = std::chrono::high_resolution_clock::now();
#else
#define START_CLOCK
#endif

#ifdef MEASURE_EXEC_TIME
#define END_CLOCK(name) t2 = std::chrono::high_resolution_clock::now();\
duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();\
                        LOGGING_INFO(lanedetectionLogger, "Execution time " << name << ": "\
                                                            << duration << endl );
#else
#define END_CLOCK(name)
#endif


/*********************************************************************/
/*************************** Debug View ******************************/
/*********************************************************************/
// #define DEBUG_ACTIVE
#define STEPPING_DISABLED 0

namespace oadrive
{
namespace lanedetection
{

#ifdef DEBUG_ACTIVE
class VotingSpace;

class RoadPerception;

class RoadPatcher;


// violet
#define COL_REF_PATCH cv::Scalar(128,0,128)

// yellow

// steel blue
#define COL_NEW_PATCH cv::Scalar(238, 172, 92)
// dark yellow
#define COL_NEW_PATCH_PREV cv::Scalar(37, 193, 255)
// red
#define COL_NEW_PATCH_PRED cv::Scalar(0, 0, 200)

// green
#define COL_ROI cv::Scalar(0, 139, 0)

#define COL_MAX_LOCATION cv::Scalar(200, 0, 0)
#endif
class HaarFilter;

class DebugViewRoadPerception
{

public:
  static void setHaarFilter(HaarFilter* hf);

  static cv::Mat generateFeatureDebugImage(const FeaturePointerVector features);

private:
  static HaarFilter* haarFilterToDebug;

#ifdef DEBUG_ACTIVE
public:

  static void setVotingSpace(const VotingSpace* vs);

  static void setRoadPerception(RoadPerception* rp);

  static void setRoadPatcher(const RoadPatcher* sp);

  static cv::Mat* generateHoughSpaceDebugImage(const VotingSpace* votingSpaceToDebug,
                                                 const oadrive::lanedetection::RoadPatcher* patcher);


  static void visualizePatches(PatchHypothesisList& patchHyps);

private:

  static void visualizeAngleRegion(cv::Mat &dbgImage, int angleRegion,
                                    cv::Size2f offset);

  static void visualizeAngleRegionVotes(cv::Mat &dbgImage, int angleRegion,
                                         cv::Size2f offset);


  static void drawPolygon(cv::Mat &image, util::Polygon polygon, cv::Scalar color,
                            int thickness);

  static void visualizeInBirdview(cv::Mat &debugImage);

  static void resetOtherClasses();


private:

  static cv::Mat debugImage;
  static float debugScale;

  static const VotingSpace* mVotingSpaceToDebug;
 // static IntersectionVotingSpace* votingSpaceIntersectionToDebug;
  static const RoadPatcher* roadPatcherToDebug;
  static RoadPerception* roadPerceptionToDebug;


  // Properties of votingSpace
  static cv::Size2i dbgBirdviewSize;
  static unsigned short dbgRowsVS;
  static unsigned short dbgColsVS;
  static util::CoordinateConverter* dbgConverter;
  static float dbgScaleVS;

  static cv::Size2i dbgOffsetBirdviewToVS;
  static cv::Size2f dbgPatchSize;
#endif
};


}
}



#endif //PROJECT_DEBUGVIEWROADPERCPTION_H

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
 * \author  Fabian Dürr
 * \date    2015-11-01
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 * Street-Detection.
 * - Uses the HaarFilter's responses to vote for Street Patches.
 *   These patches are then added to the Environment.
 *
 */
//----------------------------------------------------------------------



#ifndef OADRIVE_LANEDETECTION_STREETPATCHER_H
#define OADRIVE_LANEDETECTION_STREETPATCHER_H

#include <oadrive_util/CoordinateConverter.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>

#include "oadrive_lanedetection/FeatureDetection/StreetTypes.h"
#include "RoadPatcher.h"
#include "PatchHypothesis.h"
#include "StreetVotingSpace.h"
#include "RoadPerceptionConfig.h"


namespace oadrive
{
namespace lanedetection
{

/*! \brief Class which should calculate street patches from features which were detected in the bird view image.
 * The class works in car-coordinates, so features which were found on the bird-view should be
 * transformed into car-space (see CoordinateConverter class).
 */
class StreetPatcher : public RoadPatcher
{

  INIT_MEASURE_EXEC_TIME
#ifdef DEBUG_ACTIVE
  friend class DebugViewRoadPerception;
#endif

public:

   /*! \brief Constructor. Needs width an height in meters.
   * The width and height will be used to generate a voting-image (similar to Hough-Space
   * of the Hough-Transform). Only votes which fall into this area will be considered.
   * Should be large enough to hold the entire bird view, the car position, and any patches
   * which have already been found and should be used for the voting again.
   * \param birdviewSize Size of area in which patches can be found (in meters)
   * \param coordConverter Pointer to a coordinate converter. Make sure to keep the
   * 		converter alive as long as the StreetPatcher exists!!
   * \note At the moment, no boundary checks are made - so make sure you only pass in features
   * 		which fit into the area given by size (i.e. size/2 around the current carPose).
   */
   StreetPatcher(cv::Size2i birdviewSize, util::CoordinateConverter* coordConverter);

  ~StreetPatcher();

  /*!
   * Update the current reference hypothesis and the angle history.
   * @param currentRefHyp the new reference patch.
   * @param angleHistory the new angle history.
   */
  void update(const PatchHypothesis &currentRefHyp, const std::list<float> angleHistory);

  /*!
   * Generates patch hypothesis from given features which represent the current belief where the
   * patches are. A voting system is used in which the features vote. A vote represents the
   * feature's beliefe where a patch center could be. The patches are than creates at maximum
   * locations in the voting space.
   * @param features which vote for patch centers and by doing so determining the found patches.
   * @param currentPatchHyps the so far found hypotheses about patches.
   * @return the newly found patch hypotheses.
   */
  PatchHypothesisList generatePatches(const FeaturePointerVector &features,
                                      const PatchHypothesisList &currentPatchHyps);

  /*!
   * Generates patch hypothesis from given features which represent the current belief where the
   * patches are. A voting system is used in which the features vote. A vote represents the
   * feature's beliefe where a patch center could be. The patches are than creates at maximum
   * locations in the voting space.
   * @param features which vote for patch centers and by doing so determining the found patches.
   * @param currentPatchHyps the so far found hypotheses about patches.
   * @return the newly found patch hypotheses.
   */
  PatchHypothesisList generateCrossPatches(const FeaturePointerVector &features,
                                           const PatchHypothesisList &currentPatchHyps);


  /*!
   * Definines how many patches will be searched by generatePatchesFromFeatures().
   * @param numOfPatches the number of patches searched for.
   */
  void setNumberOfPatches(int numOfPatches);


private:

  void findPatches(const PatchHypothesisList &currentPatchHyps);

  void findCrossPatches(const PatchHypothesisList &currentPatchHyps);

  float predictAngle(float baseAngle);

  void initFeatureVotes();

  void initPatchVotes();

  void initCrossPatchVotes();

  /*! \brief Add newly found features to the class. Will be sorted by angle.*/
  void addFeatures(const FeaturePointerVector& features);

  /*! \brief Add a single feature. Will be sorted by angle.*/
  void addFeature(const Feature* f);

  /*! \brief Add a single patch vote. Will be sorted by its angle.*/
  void addPatchHypothesis(const PatchHypothesis &patchHyp);

  bool verifyCrossPatch(const cv::Mat &neuralNetOutput,
                          const PatchHypothesis &crossPatchHyp);


/******************************************************************************/
/**************************** class attributes ********************************/
/******************************************************************************/

  /*! The voting space the votes of the features are placed in */
  StreetVotingSpace mVotingSpace;

  int mNumOfPatches;

  /*! The current reference patch */
  PatchHypothesis mCurrentRefHyp;

  // Array with the angle differences between the last 6 patches.
  std::list<float> mAngleHistory;

  cv::Mat mNeuralNetOutput;

  bool mNNetOutputProcessed;

#ifdef DEBUG_ACTIVE
 virtual void dbgVisualizeVotingSpace() override;
#endif

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}    // namespace
}    // namespace

#endif

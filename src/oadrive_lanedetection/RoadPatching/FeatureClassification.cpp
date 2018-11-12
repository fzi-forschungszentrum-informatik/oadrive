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

#include "FeatureClassification.h"

using icl_core::logging::endl;
using icl_core::logging::flush;


namespace oadrive
{
namespace lanedetection
{

FeatureClassification::FeatureClassification(cv::Size birdViewSize,
                                             util::CoordinateConverter* coordConverter) :
        mCoordConverter(coordConverter),
        mClassificationPatcher(birdViewSize, coordConverter),
        mMergeMode(false)
{
}


/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/

void FeatureClassification::classifyFeatures(FeaturePointerVector features,
                                             PatchHypothesis currentRefHypothesis,
                                             PatchHypothesisList currentPatchHypotheses, bool reset)
{
  double range = 0.4;
  uint numIterations = 4;

  mClassificationPatcher.setNumberOfPatches(1);

  PatchHypothesis localRefHyp = currentRefHypothesis;
  FeaturePointerVector classifiedFeatures;
  for (uint i = 0; i < numIterations; i++)
  {
    // Transform intersections to act like a normal patch:
    if (localRefHyp.getPatchType() == PatchHypothesis::Type::INTERSECTION) {
      localRefHyp.setX(localRefHyp.getX() + 0.5 * cos(localRefHyp.getYaw()));
      localRefHyp.setY(localRefHyp.getY() + 0.5 * sin(localRefHyp.getYaw()));
    }
    transformFeatures(features, localRefHyp.getPose(), reset);

    FeaturePointerVector newlyClassified = classifyNearestFeatures(features, range);

    for (Feature* f : newlyClassified)
    {
      classifiedFeatures.push_back(f);
    }

    // Search for a patch based on the classified features
    mClassificationPatcher.update(localRefHyp, std::list<float>(0));
    PatchHypothesisList newRefPatch =
            mClassificationPatcher.generatePatches(classifiedFeatures,
                                                   PatchHypothesisList());

    localRefHyp = newRefPatch.front();

    // Check if the patch is based on pure prediction
    if (!(localRefHyp.getBasedOn() == PatchHypothesis::BasedOn::FEATURE || localRefHyp.getBasedOn() == PatchHypothesis::BasedOn::MAP))
    {
      // If so look if there is a other hypothesis with the same ID which is feature based
      for (PatchHypothesis patchHyp : currentPatchHypotheses)
      {
        if (patchHyp.getPatchID() == localRefHyp.getPatchID())
        {
          localRefHyp = patchHyp;
        }
      }

      // If still pure predicted stop classifying
      if (!(localRefHyp.getBasedOn() == PatchHypothesis::BasedOn::FEATURE || localRefHyp.getBasedOn() == PatchHypothesis::BasedOn::MAP))
      {
        break;
      }
    }
  }
}

void FeatureClassification::adjustPatchPositions(const FeaturePointerVector &featurePointer,
                                                 PatchHypothesis currentRefPatchHyp,
                                                 PatchHypothesisList currentPatchHyps)
{
  FeaturePointerVector leftLane;
  FeaturePointerVector centerLane;
  FeaturePointerVector rightLane;

  for (Feature* f : featurePointer)
  {
    if (f->type == RIGHT_SIDE_LINE)
    {
      rightLane.push_back(f);
    }
    else if (f->type == LEFT_SIDE_LINE)
    {
      leftLane.push_back(f);
    }
    else if (f->type == CENTER_LINE)
    {
      centerLane.push_back(f);
    }
  }

  float offset = calculatePositionOffset(leftLane, centerLane, rightLane);

  float translateX = cos(currentRefPatchHyp.getYaw() + M_PI_4) * offset;
  float translateY = sin(currentRefPatchHyp.getYaw() + M_PI_4) * offset;

  currentRefPatchHyp.setX(currentRefPatchHyp.getX() + translateX);
  currentRefPatchHyp.setY(currentRefPatchHyp.getY() + translateY);

  for (auto patchHyp = currentPatchHyps.begin(); patchHyp != currentPatchHyps.end(); patchHyp++)
  {
    (*patchHyp).setX((*patchHyp).getX() + translateX);
    (*patchHyp).setY((*patchHyp).getY() + translateY);
  }
}


/******************************************************************************/
/***************************** private methods *********************************/
/******************************************************************************/

void FeatureClassification::transformFeatures(FeaturePointerVector &featurePointer,
                                              OadrivePose refPose, bool reset)
{
  for (uint i = 0; i < featurePointer.size(); i++)
  {
    featurePointer.at(i)->poseRelPatch =
            mCoordConverter->world2Car(refPose, featurePointer.at(i)->localPose);

    // Overwrite the classification from the feature detection. At the moment we are overwriting all
    // classifications because they are very unreliable.
    // @Future: Improve classification in feature detection and comment then use the if clause here.
    //if (featurePointer.at(i)->type == CENTER_LINE || featurePointer.at(i)->type == LEFT_SIDE_LINE)
    if (reset)
    {
      featurePointer.at(i)->type = STREET_LINE;
    }
  }
}

FeaturePointerVector
FeatureClassification::classifyNearestFeatures(FeaturePointerVector &featurePointer,
                                               float range)
{
  FeaturePointerVector relevantFeatures;
  for (auto it = featurePointer.begin(); it != featurePointer.end();)
  {
    float localYawAbs = std::abs((*it)->poseRelPatch.getYaw());

    // only street line features, closer than range and with a relative angle smaller than
    // abs(PI / 4) are considered.
    if ((*it)->type != STREET_LINE || (*it)->poseRelPatch.getX() > (mMergeMode?2.5:1.0)*range ||
        (localYawAbs > M_PI_4 && localYawAbs < 7 * M_PI_4))
    {
      it++;
      continue;
    }

    // classify based on the local y of the feature referring to the patch
    float lateralPosition = (*it)->poseRelPatch.getY();
    
    if (!mMergeMode) {
      if (-1.5 * LANE_WIDTH < lateralPosition && lateralPosition < -0.5 * LANE_WIDTH)
      {
        (*it)->type = RIGHT_SIDE_LINE;
      }
      else if (0.5 * LANE_WIDTH < lateralPosition && lateralPosition < 1.5 * LANE_WIDTH)
      {
        (*it)->type = LEFT_SIDE_LINE;
      }
      else if (-0.5 * LANE_WIDTH <= lateralPosition && lateralPosition <= 0.5 * LANE_WIDTH)
      {
        (*it)->type = CENTER_LINE;
      }
    }
    // behave slightly differently is merge mode is active
    else {
      // increase range where right side lines are searched 
      if (-2 * LANE_WIDTH < lateralPosition && lateralPosition < 0 * LANE_WIDTH)
      {
        (*it)->type = RIGHT_SIDE_LINE;
      }
      // ignore everything else
      else {
        it++;
        continue;
      }
    }

    relevantFeatures.push_back((*it));
    featurePointer.erase(it);
  }

  return relevantFeatures;
}

float FeatureClassification::calculatePositionOffset(FeaturePointerVector &leftLine,
                                                     FeaturePointerVector &centerLine,
                                                     FeaturePointerVector &rightLine) const
{
  std::vector<float> validMeans;
  std::vector<float> scores;
  std::vector<uint> sizes;
  std::vector<float> lineOffsets;

  if (addMeanAndStdDev(leftLine, validMeans, scores, sizes))
  {
    lineOffsets.push_back(HALF_ROAD_WIDTH);
  }
  if (addMeanAndStdDev(centerLine, validMeans, scores, sizes))
  {
    lineOffsets.push_back(0.f);
  }
  if (addMeanAndStdDev(rightLine, validMeans, scores, sizes))
  {
    lineOffsets.push_back(-HALF_ROAD_WIDTH);
  }

  float summedWeights = 0.f;
  for (uint i = 0; i < scores.size(); i++)
  {
    summedWeights += sizes.at(i) * scores.at(i);
  }

  std::vector<float> weights;
  for (uint i = 0; i < scores.size(); i++)
  {
    weights.push_back(sizes.at(i) * scores.at(i) / summedWeights);
  }

  /* minimize:   weightLeft * (meanLeft - lanewidth + offset)²
   *           + weightCenter * (meanLeft - 0 + offset)²
   *         + weightRight * (meanRight + lanewidth + offset)²
   *
   * derivation:   weightLeft * (2 * (meanLeft - lanewidth) + offset)
   *           + weightCenter * (2 * meanCenter + offset)
   *            + weightRight * (2 * (meanRight + lanewidth) + offset)
   *
   * offset = - (2 * weightLeft * (meanLeft - lanewidth) + 2 * weightCenter * meanCenter + 2 *
   *             weightRight * (meanRight + lanewidth))
   */

  float offset = 0.f;
  for (uint i = 0; i < validMeans.size(); i++)
  {
    offset += weights.at(i) * (validMeans.at(i) - lineOffsets.at(i));
  }

  return offset;
}

bool FeatureClassification::addMeanAndStdDev(FeaturePointerVector &line,
                                             std::vector<float> &validMeans,
                                             std::vector<float> &scores,
                                             std::vector<uint> &sizes) const
{
  const float thresholdStdDev = 0.1f;

  if (line.size() > 1)
  {
    float meanLeft = calculateFeatureYMean(line, 0.1f);
    float stdDevLeft = calculateFeatureYStdDev(line, meanLeft);

    // to rely on the y mean of the line the standard deviation has to be below a given threshold.
    if (stdDevLeft < thresholdStdDev)
    {
      validMeans.push_back(meanLeft);
      scores.push_back(thresholdStdDev - stdDevLeft);
      sizes.push_back(line.size());
      return true;
    }

    return false;
  }

  return false;
}

float FeatureClassification::calculateFeatureYMean(FeaturePointerVector &featurePointer,
                                                   float intervalSize) const
{
  float sum = 0.f;
  for (const Feature* f : featurePointer)
  {
    sum += f->poseRelPatch.getY();
  }

  if (featurePointer.size() > 0)
  {
    float initialMean = sum / float(featurePointer.size());
    sum = 0.f;
    int numRelevantFeatures = 0;
    // search for features whose y-position is more than intervalSize from the calculated mean away
    // recalculate the mean ignoring those features.
    for (auto it = featurePointer.begin(); it != featurePointer.end();)
    {
      if (std::fabs((*it)->poseRelPatch.getY() - initialMean) < intervalSize)
      {
        sum += (*it)->poseRelPatch.getY();
        numRelevantFeatures++;
        it++;
      }
      else
      {
        it = featurePointer.erase(it);
      }
    }

    return sum / float(numRelevantFeatures);
  }
  else
  {
    return 0.f;
  }
}

float FeatureClassification::calculateFeatureYStdDev(const FeaturePointerVector &featurePointer,
                                                     float mean) const
{
  float sum = 0.f;
  for (const Feature* f : featurePointer)
  {
    sum += std::pow((f->poseRelPatch.getY() - mean), 2);
  }

  if (featurePointer.size() > 0)
  {
    return std::sqrt(sum / float(featurePointer.size()));
  }
  else
  {
    return std::numeric_limits<float>().max();
  }
}

bool FeatureClassification::insideInvalidMask(const Feature &feature)
{
  // size of the engine cover in the birdview image.
  cv::Size2i engineCoverSize(330, 30);

  cv::Point2f featureImagePos = mCoordConverter->car2Pixel(feature.localPose);
  cv::Size2i imageSize = mCoordConverter->getImgSizeBirdView();

  if ((featureImagePos.x > (imageSize.width - engineCoverSize.width) * 0.5) &&
      (featureImagePos.x < (imageSize.width + engineCoverSize.width) * 0.5) &&
      (featureImagePos.y > imageSize.height - engineCoverSize.height))
  {
    return true;
  }


  return false;
}

void FeatureClassification::setMergeMode(bool mergeModeActive) {
  mMergeMode = mergeModeActive;
}

}
}
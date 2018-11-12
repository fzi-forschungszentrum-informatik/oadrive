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
 */
//----------------------------------------------------------------------

#include <oadrive_lanedetection/lanedetectionLogging.h>
#include "VotingSpace.h"

#include <math.h>

using oadrive::util::ImageHelper;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace lanedetection
{

VotingSpace::VotingSpace(util::CoordinateConverter* coordConverter, cv::Size2i size) :
        mScale(0.25),
        mMeters2Pixels(coordConverter->getPixelsPerMeterX(mScale),
                       coordConverter->getPixelsPerMeterY(mScale)),
        mSize(size),
        mRows(mSize.height * mScale + 0.8 * mMeters2Pixels.y),
        mCols(mSize.width * mScale + 0.8 * mMeters2Pixels.x),
        mCoordConverter(coordConverter),
        mCenterPixel(mCols * 0.5, mRows * 0.5),
        mCenterPose(coordConverter->pixel2Car2(cv::Size2f(size.width * 0.5, size.height * 0.5))),
        mNumAngleRegions(40),         // Divide voting space into angle regions
        mAngleRegionSize(2.0 * M_PI / mNumAngleRegions),
        mBlurSize(int(0.04 * mMeters2Pixels.x) % 2 == 0 ?
                  int(0.04 * mMeters2Pixels.x) + 1 :
                  int(0.04 * mMeters2Pixels.x))
{
  LOGGING_INFO(lanedetectionLogger, "" << "Generating Voting Space: " << endl
                                       << "\tSize: " << mRows / mMeters2Pixels.x << "m : "
                                       << mCols / mMeters2Pixels.y << "m" << endl
                                       << "\tSize: " << mCols << "px : " << mRows << "px" << endl
                                       << "\tCenterPose world: " << mCenterPose.getX() << " : "
                                       << mCenterPose.getY() << endl
                                       << "\tCenterPose space: " << mCenterPixel.x << "px : "
                                       << mCenterPixel.y << "px" << endl
                                       << "\tAngle Regions: " << mNumAngleRegions << endl
                                       << "\tAngle region size: " << mAngleRegionSize << endl
                                       << "\tScale: " << mScale << endl);

};

/******************************************************************************/
/***************************** public methods *********************************/
/******************************************************************************/

void VotingSpace::addVote(const OadrivePose &pose, const Vote &vote, VotingContainer* votingSpace)
{
  float featureAngle = pose.getYaw();
  float ang = featureAngle + vote.angleOffset;

  // Determine which angle map this patch should vote into:
  uint angleRegion = angle2AngleRegionID(ang);

  // Rotate vote according to patch angle, then add to Point (x,y):
  double xVote, yVote;
  xVote = pose.getX() + cos(featureAngle) * vote.pos.x - sin(featureAngle) * vote.pos.y;
  yVote = pose.getY() + sin(featureAngle) * vote.pos.x + cos(featureAngle) * vote.pos.y;

  OadrivePose votePose(xVote, yVote, 0.f);

  // discretize the pose position and place it into the corresponding pixel:
  cv::Point2f imagePos = mCoordConverter->car2VotingSpace(mCenterPose, mCenterPixel, votePose,
                                                          mScale);

  cv::Size2f votingSize;
  votingSize.width = vote.size.width;
  votingSize.height = vote.size.height;
  cv::RotatedRect rr(cv::Point2f(imagePos.x, imagePos.y), votingSize, -featureAngle * 180.0 / M_PI);
  util::Polygon polygon = mCoordConverter->scaleRectangle(rr, mScale);
  if (angleRegion >= 0 && angleRegion < votingSpace->regions.size())
  {
    votingSpace->regionWasUsed.at(angleRegion) = true;
    ImageHelper::addPolygon(votingSpace->regions.at(angleRegion), polygon, vote.weight);
  }
}

/******************************************************************************/
/**************************** private methods *********************************/
/******************************************************************************/

VotingSpace::MaxVote
VotingSpace::findMaxVote(const cv::Mat &votingSpaceA, const cv::Mat &votingSpaceB) const
{
  assert(votingSpaceA.rows == votingSpaceB.rows && votingSpaceA.cols == votingSpaceB.cols);

  MaxVote max;
  for (int rows = 0; rows < votingSpaceA.rows; rows++)
  {
    for (int cols = 0; cols < votingSpaceA.cols; cols++)
    {
      ushort minVal = std::min(votingSpaceA.at<ushort>(rows, cols),
                               votingSpaceB.at<ushort>(rows, cols));

      if (minVal > max.value)
      {
        max.value = minVal;
        max.location = cv::Point2i(cols, rows);
      }
    }
  }

  return max;
}

float VotingSpace::calculateAngle(const std::vector<cv::Mat> &votingSpace, int maxRegion,
                                  cv::Point2i imgPosMax) const
{
  float totalScore = 0.0;
  float angleWeightedSum = 0.0;

  // Calculate the weighted angle sum over the consideres angle regions.
  for (int i = -CONSIDERED_NEIGBOUR_REGIONS; i <= +CONSIDERED_NEIGBOUR_REGIONS; i++)
  {
    int angleReg = getNeighbourAngleRegion(maxRegion, i);

    ushort weight = votingSpace.at(angleReg).at<ushort>(imgPosMax.y, imgPosMax.x);

    float angle = angleRegion2Angle(angleReg);
    angleWeightedSum += angle * float(weight);

    totalScore += weight;
  }

  float angle = 0.f;
  if (totalScore >= std::numeric_limits<float>::min())
  {
    angle = angleWeightedSum / totalScore;
  }

  return angle;
}

float VotingSpace::angleRegion2Angle(int regionID) const
{
  float angleRegionSize = 2.0 * M_PI / mNumAngleRegions;
  return angleRegionSize * (regionID - (mNumAngleRegions / 2));
}

uint VotingSpace::angle2AngleRegionID(float angle) const
{
  // Move by half a region (so that 0 is center of region)
  angle = angle + mAngleRegionSize * 0.5;

  // Wrap into -M_PI...M_Pi range:
  angle = std::fmod(angle, 2.f * M_PI);

  if (angle >= M_PI) angle -= 2.f * M_PI;
  if (angle < -M_PI) angle += 2.f * M_PI;

  int regionID = floor(angle / M_PI * mNumAngleRegions / 2) + (mNumAngleRegions / 2);

  assert(regionID >= 0 && regionID < mNumAngleRegions);

  return uint(regionID);
}

int VotingSpace::getNeighbourAngleRegion(int regionID, int offset) const
{
  regionID += offset;

  while (regionID >= mNumAngleRegions)
  {
    regionID -= mNumAngleRegions;
  }
  while (regionID < 0)
  {
    regionID += mNumAngleRegions;
  }

  if (regionID >= mNumAngleRegions)
  {
    regionID -= mNumAngleRegions;
  }

  return regionID;
}

bool VotingSpace::calculateRelevantVotingSpace(const cv::Mat &votingSpace,
                                               const RegionOfInterest &regionOfInterest,
                                               cv::Mat &relevantVotingSpace)
{

  util::Polygon roiAsPolygon = convertRoiToPolygon(regionOfInterest);
  cv::Rect boundingBox = cv::boundingRect(roiAsPolygon.vertices);
  mRelevantRoi = boundingBox;
  cv::Mat mask = ImageHelper::createMask(roiAsPolygon, boundingBox);

  if (!ImageHelper::isValidRectangle(votingSpace, boundingBox))
  {
    // If not valid, resize the roi so it fits in the voting space.
    cv::Rect intersection = cv::Rect(0, 0, votingSpace.cols, votingSpace.rows)
                            & boundingBox;

    if (intersection.height <= 0 || intersection.width <= 0)
    {
      return false;
    }
    mRelevantRoi = intersection;
    cv::Mat tmp = votingSpace(intersection);
    cv::Mat selectedRegion(boundingBox.size(), CV_16UC1, cv::Scalar(0));
    cv::Point2f offset = ImageHelper::calculateInvalidRoiOffset(boundingBox, votingSpace);
    tmp.copyTo(selectedRegion(cv::Rect(offset.x, offset.y, tmp.cols, tmp.rows)));
    cv::blur(selectedRegion, selectedRegion, cv::Size(mBlurSize, mBlurSize));
    relevantVotingSpace = selectedRegion.mul(mask);
    relevantVotingSpace = relevantVotingSpace(cv::Rect(offset.x, offset.y, tmp.cols, tmp.rows));
  }
  else
  {
    cv::Mat selectedRegion = votingSpace(boundingBox);
    cv::blur(selectedRegion, selectedRegion, cv::Size(mBlurSize, mBlurSize));
    relevantVotingSpace = selectedRegion.mul(mask);
  }

  return true;
}

util::Polygon VotingSpace::convertRoiToPolygon(RegionOfInterest regionOfInterest)
{
  cv::Point2f roiCenterPixel = mCoordConverter->car2VotingSpace(mCenterPose, mCenterPixel,
                                                                regionOfInterest.center, mScale);

  cv::Size2f roiSize(regionOfInterest.width, regionOfInterest.length);

  // Create an orientated rectangle which represents the region of interest
  cv::RotatedRect rectangle(roiCenterPixel, roiSize,
                            -regionOfInterest.center.getYaw() * 180.0 / M_PI);

  return mCoordConverter->scaleRectangle(rectangle, mScale);
}

cv::Point2f
VotingSpace::transformToGlobalVotingSpace(cv::Point2f localPixelCoords,
                                          const RegionOfInterest &regionOfInterest) const
{
  float x = mRelevantRoi.x + localPixelCoords.x;
  float y = mRelevantRoi.y + localPixelCoords.y;

  return cv::Point2f(x, y);
}


}
}
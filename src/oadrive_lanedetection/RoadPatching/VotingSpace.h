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

#ifndef OADRIVE_LANEDETECTION_VOTINGSPACE_H
#define OADRIVE_LANEDETECTION_VOTINGSPACE_H

#include <chrono>

#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_util/ImageHelper.h>

#include "oadrive_lanedetection/FeatureDetection/StreetTypes.h"
#include "oadrive_lanedetection/DebugViewRoadPerception.h"

#include "PatchHypothesis.h"


namespace oadrive
{
namespace lanedetection
{

typedef unsigned short ushort;
typedef unsigned int uint;

/*
 * Struct which models a 3-dimensional voting space and tracks which angle regions have been used.
 * This speeds up the rest of the voting space because only used regions must be reset.
 */
struct VotingContainer
{
  const size_t mNumAngleRegions;
  std::vector<cv::Mat> regions;
  std::vector<bool> regionWasUsed;


  VotingContainer(size_t numAngleRegions, cv::Size2i regionSize) :
          mNumAngleRegions(numAngleRegions),
          regionWasUsed(numAngleRegions, false)
  {
    for (size_t i = 0; i < mNumAngleRegions; i++)
    {
      regions.push_back(cv::Mat(regionSize.height, regionSize.width, CV_16UC1, cv::Scalar(0)));
    }

    assert(regions.size() == numAngleRegions && regionWasUsed.size() == numAngleRegions);
  }

  void reset()
  {
    assert(regions.size() == mNumAngleRegions && regionWasUsed.size() == mNumAngleRegions);

    for (unsigned short i = 0; i < mNumAngleRegions; i++)
    {
      // only reset used regions
      if (regionWasUsed.at(i))
      {
        regions.at(i) = cv::Scalar(0);
        regionWasUsed.at(i) = false;
      }
    }
  }
};

class VotingSpace
{

#ifdef DEBUG_ACTIVE
  friend class DebugViewRoadPerception;
#endif

public:

  /*
   * Encapsulates the information of a maximum in the voting space. It contains the value, location
   * and angle.
   */
  struct MaxVote
  {
    ushort value;
    cv::Point2i location;
    float angle;
    PatchHypothesis::BasedOn basedOn;

    MaxVote() : value(0), location(cv::Point2i(0, 0)), angle(0.f) {}
  };

public:

  VotingSpace(util::CoordinateConverter* coordConverter, cv::Size2i size);

  /*!
   * Clears all votes.
   * */
  virtual void clear() = 0;

  /*!
   * Get the size of a angle region.
   * @return the size of a angle region.
   */
  float getAngleRegionSize() { return mAngleRegionSize; }

  cv::Size2i getVotingSpaceSize() { return cv::Size2i(mCols, mRows); }

  /*!
    * Creates a rectangle out of the given region of interest based on the center location
    * and size of the roi.
    * @param roi the region of interest for which the rectangle should be created.
    * @return the created rotated rectangle representing the roi.
    */
  util::Polygon convertRoiToPolygon(RegionOfInterest roi);

protected:

  /*!
 * Add a vote to the forwarded voting space specified by a detected feature and a vote belonging
 * to that feature type.
 * @param pose The pose of the feature which is responsible for the vote
 * @param vote The vote which should be placed in the voting space.
 * @param votingSpace The voting space the vote should be added to.
 */
  void addVote(const OadrivePose &pose, const Vote &vote, VotingContainer* votingSpace);

  bool calculateRelevantVotingSpace(const cv::Mat &votingSpace,
                                    const RegionOfInterest &regionOfInterest,
                                    cv::Mat &relevantVotingSpace);

  /*!
   * Find the position which min(votingSpaceA(i), votingSpaceB(i)) is maximal.
   * @param votingSpaceA the first votingSpace.
   * @param votingSpaceB the second votingSpace.
   * @return the found maximum over the minimum of A and B.
   */
  MaxVote findMaxVote(const cv::Mat &votingSpaceA, const cv::Mat &votingSpaceB) const;

  /*!
   * Transforms the pixelCoords which refer to a local region of interest back to the global
   * voting space.
   * @param localPixelCoords the local pixel coordinate refering to a local region of interest.
   * @param regionOfInterest the region of interest the local coordinates refering to.
   * @return the global pixel coordinates.
   */
  cv::Point2f transformToGlobalVotingSpace(cv::Point2f localPixelCoords,
                                           const RegionOfInterest &regionOfInterest) const;

  /*!
   * Calculates the weighted mean angle based on the forwarded voting space and the position.
   * Starting CONSIDERED_ANGLE_REGIONS left of maxRegion and ending CONSIDERED_ANGLE_REGIONS
   * right of maxRegion. Fore each region the angle is weighted with the score at the position
   * of the maximum. The final sum is divided by the total weight/score.
   * @param votingSpace the voting space which is the basis for the angle calculation.
   * @param maxRegion the angle region where the maximum was found.
   * @param imgPosMax the position of the maximum in pixel coordinates.
   * @return the calculated angle.
   */
  float calculateAngle(const std::vector<cv::Mat> &votingSpace, int maxRegion,
                       cv::Point2i imgPosMax) const;

  //! Discretize the angle into mNumAngleRegion regions:
  uint angle2AngleRegionID(float angle) const;

  //! Retrieve the angle from the angle region:
  float angleRegion2Angle(int regionID) const;

  //! Retrieve another angle region left or right from this one (wraps around).
  int getNeighbourAngleRegion(int regionID, int offset) const;


/******************************************************************************/
/**************************** class attributes ********************************/
/******************************************************************************/

  /*! The scale of the voting space. Specifies the size of the real world area which
   * is maped to one cell in the voting space.
   * -> Real world coordinates are scaled by this factor */
  float mScale;

  //! the factor for converting meters to pixel
  cv::Point2f mMeters2Pixels;

  // Size of the voting space in meters (x for width and y for height, to be consistent with images)
  cv::Size2i mSize;
  //! Height of Hough space in pixel
  unsigned short mRows;
  //! Width of Hough space in pixel
  unsigned short mCols;

  util::CoordinateConverter* mCoordConverter;

  //! Center of image in pixels:
  cv::Point2i mCenterPixel;

  /*! The center pose of the voting space in global coordinates. */
  OadrivePose mCenterPose;

  const unsigned short CONSIDERED_NEIGBOUR_REGIONS = 2;
  unsigned short mNumAngleRegions;
  float mAngleRegionSize;

  // The size of the blurring operation in meters.
  int mBlurSize;

  cv::Rect mRelevantRoi;

protected:

#ifdef DEBUG_ACTIVE
  bool dbgHasROI;
  int dbgCurrentAngleRegion = 0;
  int dbgValueOfMax = 0.0;
  cv::Point2f dbgPosOfMax;
  int dbgMaxAngleRegion = 0;
  RegionOfInterest dbgCurrentRegion;
  cv::RotatedRect dbgCurrentROI;
  std::vector<const VotingContainer*> dbgRegionsToDebug;
#endif

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}
}

#endif //OADRIVE_LANEDETECTION_VOTINGSPACE_H

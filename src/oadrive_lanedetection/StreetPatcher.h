// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2016 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-01
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
#include <oadrive_world/Environment.h>
#include <oadrive_world/Patch.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "StreetTypes.h"
#include "HaarFilter.h"

namespace oadrive {
namespace lanedetection {

enum enumAPrioriStrength
{
  STRENGTH_LOW, STRENGTH_MEDIUM, STRENGTH_HIGH
};

/*! \brief Class which should calculate street patches from features which were detected in the bird view image.
 * The class works in car-coordinates, so features which were found on the bird-view should be
 * transformed into car-space (see CoordinateConverter class).
 */
class StreetPatcher
{
public:

  /*! \brief Constructor. Needs width an height in meters.
   * The width and height will be used to generate a voting-image (similar to Hough-Space
   * of the Hough-Transform). Only votes which fall into this area will be considered.
   * Should be large enough to hold the entire bird view, the car position, and any patches
   * which have already been found and should be used for the voting again.
   * \param size Size of area in which patches can be found (in meters)
   * \param coordConverter Pointer to a coordinate converter. Make sure to keep the
   * 		converter alive as long as the StreetPatcher exists!!
   * \note At the moment, no boundary checks are made - so make sure you only pass in features
   * 		which fit into the area given by size (i.e. size/2 around the current carPose).
   */
  StreetPatcher( float size,
                 oadrive::util::CoordinateConverter* coordConverter,
                 bool letCenterVoteAsSide = false,
                 bool letSideVoteAsCenter = false );

  ~StreetPatcher();

  /*! Resets the StreetPatcher. This might be needed after turning at a cross section.
   * \note To re-initialize fully, the Environment's Patches also have to be cleared by
   * 		calling Environment::clearAllPatrches(). Otherwise patch-votes from previously found
   * 		patches will still be used (which is okay if that's what you want). */
  void reset();

  /*! \brief Attempt to calculate new patches from the current features.
         * \param maximumNum Maximum number of patches after which the algorithm will stop looking. */
  void findPatches( unsigned int maximumNum = 1 );

  /*! \brief Show the patches on an image. */
  cv::Mat generateDebugImage();

  /*! \brief Return the hough transfrom image. */
  cv::Mat generateHoughSpaceDebugImage();

  /* \brief Returns a pointer to all the Patches. Hopefully the correct ones :)*/
  PatchPtrList* getPatches() { return &mPatches; }

  /*! Resets the algorithm, removes any previously added features. */
  void clearFeatures();

  /*! Removes any previously found patches. */
  void clearPatches();

  /*! Call for every image you want to be interpreted.
   * Will pass the image on to the HaarFilter to find features, will then use voting
   * on those features to find patches.
   * Will then automatically add these patches to the environment. */
  void setImage( cv::Mat image, const oadrive::core::ExtendedPose2d &carPose );

  void setSearchCrossSections( bool search );
  void setSearchParkingLots( bool search );

  /*! Set a vote by using some a-priori knowledge:
   * This vote can be set when you know approximately where the street should be.
   * \param pose The position of where you believe the Street to be. A lengthly vote
   * 		will be cast into the corresponding hough-space. */
  void setAPrioriVote( const oadrive::core::ExtendedPose2d &pose,
      enumAPrioriStrength strength = STRENGTH_HIGH );

  bool getIsInitialized() { return mHaarFilter.getInitialized(); }


  /*! Estimates the type of parking lot we're currently in
   * This looks at all the features found for the current image and determines the
   * average direction. If the average direction is towards the right, we assume the parking
   * lot is a PARKING_TYPE_CROSS, otherwise it's a PARKING_TYPE_PARALLEL.
   * \note This returns PARKING_TYPE_UNKNOWN if the StreetPatcher is not yet initialized.*/
  ParkingType getParkingSpaceDirection( cv::Mat& image);


  /*! Set the current frames per meter ( = frameRate/speed )
   * This allows the historic votes to be used the correct amount of times.
   * \note Should be called whenever speed or framerate change (or before calling setImage)! */
  //void setFramesPerMeter( float framesPerMeter );

  bool hasFirstAPrioriVote() { return mAPrioriInfoGiven; }

private:

  /*! \brief Sets the current position of the car. */
  void setCarPose( const oadrive::core::ExtendedPose2d &carPose );

  void initFeatureVotes();
  void initPatchVotes();

  void insertAPrioriVote();

  /*! \brief Add newly found features to the class. Will be sorted by angle.*/
  void addFeatures( FeatureVector* features );

  /*! \brief Add a single feature. Will be sorted by angle.*/
  void addFeature( Feature &f );

  void addPreviousPatch( oadrive::world::PatchPtr patch, bool voteForSpecialPatches = true );

  /*! \brief Add a single patch vote. Will be sorted by its angle.*/
  void insertPatchVote( oadrive::world::PatchPtr patch, bool voteForSpecialPatches = true );

  void insertTrafficSign( TrafficSignPtr trafficSign );

  void addRectangle( cv::Mat &mat, const cv::RotatedRect &rRect,
                     const cv::Rect &boundings, cv::Scalar amount );


  /*! \brief Generate images of the patches so they can be placed on the final image. */
  //void initStreetPatchImages();

  //! Discretize the angle into mNumAngleRegion regions:
  int angle2AngleRegionID( float angle );

  //! Retrieve the angle from the angle region:
  float angleRegion2Angle( int regionID );

  //! Retrieve another angle region left or right from this one (wraps around).
  int getNeighbourAngleRegion( int regionID, int offset );

  //void rotateImg( cv::Mat& src, cv::Mat& dst, double angle );

  //! Sets a rectangle of size around point p in the hough space angleRegion to newAmount.
  void setRegion( int angleRegion, cv::Point p, float size, int newAmount = 0 );

  oadrive::util::CoordinateConverter* mCoordConverter;

  unsigned short mNumAngleRegions;
  unsigned short mNumUsedAngleRegions;
  float mAngleRegionSize;
  float mScale;
  float mMeters2Pixels;
  //! Size of the region (in meters) which should be looked at in Hough-space.
  float mSize;
  //! Half of the size.
  float mRadius;
  //! Height of Hough space
  unsigned short mRows;
  //! Width of Hough space
  unsigned short mCols;
  //! Center of image in pixels:
  unsigned short mCenter;

  //! Minimum amount of line features to calculate parking lot orientation of car
  unsigned int mMinimumNumberFeaturesForParkingLotOrientation;

  int mBlurSize;    // in meters
  int mMinimumVote;
  int mMinimumVoteForDetection;
  int mMinimumVoteForDetectionCrossSection;
  int mMinimumVoteForDetectionParkingLot;

  std::vector<cv::Mat> mHoughSpace;
  std::vector<cv::Mat> mHoughSpaceForCrossSection;
  std::vector< std::vector<cv::Mat> > mHoughSpaceForParkingLot;
  cv::Mat mHoughSpaceForTrafficSigns;
  cv::Mat mCombinedParkingHoughSpace;

  PatchPtrList mPatches;

  /*! Predefined feature votes for STRAIGHT patches: */
  std::vector<std::vector<Vote> > mVotes;
  /*! Predefined feature votes for CROSS_SECTION patches: */
  std::vector<std::vector<Vote> > mVotesForCrossSection;
  /*! Predefined feature votes for PARKING_SLOT patches: */
  std::vector<std::vector<Vote> > mVotesForParkingLot;
  /*! Predefined patch votes for STRAIGHT patches: */
  std::vector<std::vector<Vote> > mPatchVotes;
  /*! Predefined patch votes for CROSS_SECTION patches: */
  std::vector<std::vector<Vote> > mPatchVotesForCrossSection;
  /*! Predefined patch votes for PARKING_SLOT patches: */
  std::vector<std::vector<Vote> > mPatchVotesForParkingLot;

  std::vector<cv::Mat> mPatchImages;

  oadrive::core::ExtendedPose2d mCenterPose;

  /*! The car pose which the car was at when the image was received. */
  oadrive::core::ExtendedPose2d mPrevCarPose;
  oadrive::core::ExtendedPose2d mImgCarPose;

  /*! Defines the region which should be cleared around a maximum vote after it
    * has been used to detect a new patch.*/
  float mClearRegionSize;

  HaarFilter mHaarFilter;

  cv::Mat mDebugHoughSpace;

  cv::Size2i mCarVoteSize;
  cv::Size2i mCarVoteSizeThin;

  bool mLetCenterVoteAsSide;
  bool mLetSideVoteAsCenter;
  float mFeatureVoteCoupling;

  bool mSearchForCrossSections;
  bool mSearchForParkingLots;

  /*! Holds votes from previous frames: */
  FeatureVector mHistoricFeatures;
  unsigned int mHistoryLength;

  oadrive::core::ExtendedPose2d mAPrioriPose;
  bool mAPrioriInfoGiven;
  enumAPrioriStrength mAPrioriStrength;

  float mCurrentFramesPerMeter;

  bool mCarIsMoving;

  bool mDebugMode;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}	// namespace
}	// namespace

#endif

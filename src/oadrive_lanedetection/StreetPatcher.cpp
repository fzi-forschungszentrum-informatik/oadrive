// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2017 FZI Forschungszentrum Informatik, Karlsruhe, Germany
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

#include "StreetPatcher.h"

#include <iostream>
#include "opencv2/opencv.hpp"
#include <boost/filesystem.hpp>
#include <oadrive_lanedetection/lanedetectionLogging.h>
#include <oadrive_world/TrafficSign.h>
#include <oadrive_world/aadc_roadSign_enums.h>

#include <oadrive_util/Config.h>

using namespace oadrive::util;
using namespace oadrive::world;

using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive {
namespace lanedetection {

StreetPatcher::StreetPatcher( float size, CoordinateConverter* coordConverter,
                              bool letCenterVoteAsSide, bool letSideVoteAsCenter )
  : mCoordConverter( coordConverter )		// copy pointer
  // Divide voting space into angle regions:
  , mNumAngleRegions(40)
  // Effectively used regions (due to patch symmetry):
  , mNumUsedAngleRegions( mNumAngleRegions/2 )
  , mAngleRegionSize( 2.0*M_PI/mNumAngleRegions )
  , mScale(0.1)   // World coordinates are scaled by this factor
  , mMeters2Pixels( mCoordConverter->getPixelsPerMeter()*mScale ) // ~334 pixels are one meter
  , mSize( size )
  , mRadius( size*0.5 )
  , mRows( size*mMeters2Pixels )
  , mCols( mRows )
  , mCenter( mRows*0.5 )
  , mMinimumNumberFeaturesForParkingLotOrientation ( 10 )
  , mBlurSize( int(0.07*mMeters2Pixels) % 2 == 0 ? int(0.07*mMeters2Pixels)+1: int(0.07*mMeters2Pixels) )
  , mMinimumVote( 1 + mBlurSize*mBlurSize )
  , mMinimumVoteForDetection( mMinimumVote*50 )		// Erfahrungswert
  , mMinimumVoteForDetectionCrossSection( mMinimumVote*50 )		// Erfahrungswert
  , mMinimumVoteForDetectionParkingLot( mMinimumVote*8 )		// Erfahrungswert
  , mVotes( numFeatureTypes, std::vector<Vote>())	// Holds votes for each type of feature
  , mVotesForCrossSection( numFeatureTypes, std::vector<Vote>())	// Holds votes for each type of feature
  , mVotesForParkingLot( numFeatureTypes, std::vector<Vote>())	// Holds votes for each type of feature
  , mPatchVotes( numPatchTypes, std::vector<Vote>())	// Holds votes for each type of patch
  , mPatchVotesForCrossSection( numPatchTypes, std::vector<Vote>())	// Holds votes for each type of patch
  , mPatchVotesForParkingLot( numPatchTypes, std::vector<Vote>())	// Holds votes for each type of patch
  , mClearRegionSize( 0.2*mMeters2Pixels )
  , mHaarFilter( mCoordConverter )
  , mCarVoteSize( 0.5*size*mMeters2Pixels, 0.7*mMeters2Pixels )
  , mCarVoteSizeThin( 0.5*size*mMeters2Pixels, 0.3*mMeters2Pixels )
  , mLetCenterVoteAsSide( letCenterVoteAsSide )
  , mLetSideVoteAsCenter( letSideVoteAsCenter )
  , mFeatureVoteCoupling( 0.2 )
  , mSearchForCrossSections( true )
  , mSearchForParkingLots( false )
  , mAPrioriInfoGiven( false )
  , mCurrentFramesPerMeter( 10.0 )	// Some initial value (should be set whenever car speed of FPS change!)
  , mCarIsMoving( false )
  , mDebugMode( Config::getBool( "General", "DebugOutput", false ))
  , mBoostCrossSectionFeatures( false )
{
  mDebugHoughSpace = cv::Mat( mRows*4, mCols*mNumUsedAngleRegions, CV_16UC1 );

  LOGGING_INFO( lanedetectionLogger, "Generating Hough Space of size: "
                << mCols << "x" << mRows << endl
                << "\tBlur Size: " << mBlurSize << " pixel" << endl
                << "\tMinimum Vote: " << mMinimumVote << endl
                << "\tMinimum Vote for Detection: " << mMinimumVoteForDetection << endl
                << "\tMinimum Vote for Cross Section Detection: " << mMinimumVoteForDetectionCrossSection << endl
                << "\tMinimum Vote for Parking Lot Detection: " << mMinimumVoteForDetectionParkingLot << endl
                << "\tCenter votes as side: " << (mLetCenterVoteAsSide ? "true" : "false") << endl
                << "\tSide votes as center: " << (mLetSideVoteAsCenter ? "true" : "false") << endl
                << "\tBird view pixels per meter: " << mCoordConverter->getPixelsPerMeter() << endl
                << "\tAngle regions: " << mNumAngleRegions << " (Using: " << mNumUsedAngleRegions << ")" << endl
                << "\tResolution: " << 1*(1/mMeters2Pixels) << "m" << endl );

  clearFeatures();
  initFeatureVotes();
  initPatchVotes();
  //initStreetPatchImages();
  boost::filesystem::create_directories( "/tmp/recordedData/Features/" );
}
StreetPatcher::~StreetPatcher()
{
}

void StreetPatcher::reset()
{
  clearFeatures();
  mHistoricFeatures.clear();
  mAPrioriInfoGiven = false;
}

void StreetPatcher::setSearchCrossSections( bool search )
{
  if( search )
    LOGGING_INFO( lanedetectionLogger, "Enabled searching for CROSS_SECTIONs." << endl );
  else
    LOGGING_INFO( lanedetectionLogger, "Disabled searching for CROSS_SECTIONs." << endl );
  mSearchForCrossSections = search;
}
void StreetPatcher::setSearchParkingLots( bool search )
{
  if( search )
    LOGGING_INFO( lanedetectionLogger, "Enabled searching for PARKING_LOT." << endl );
  else
    LOGGING_INFO( lanedetectionLogger, "Disabled searching for PARKING_LOT." << endl );
  mSearchForParkingLots = search;
}

void StreetPatcher::setCarPose( const ExtendedPose2d &carPose )
{
  Position2d diffVec = mImgCarPose.getPosition() - mPrevCarPose.getPosition();
  float dist = diffVec.norm();
  if( dist > 1e-3 )		// We've moved more than a millimeter? Then use this new pose.
  {
    mCurrentFramesPerMeter = std::min( 1.0/dist, 10.0 );
    LOGGING_INFO( lanedetectionLogger, "Current frames per meter: "
                  << mCurrentFramesPerMeter << endl );
    mPrevCarPose = mImgCarPose;
    mCarIsMoving = true;
  } else {
    LOGGING_INFO( lanedetectionLogger, "We're standing still. Disabled historic features." <<
                  endl );
    mCarIsMoving = false;
  }
  mImgCarPose = carPose;


  float dX = mRadius*cos( carPose.getYaw() );
  float dY = mRadius*sin( carPose.getYaw() );
  mCenterPose = ExtendedPose2d( carPose.getX() + dX, carPose.getY() + dY, carPose.getYaw() );

  if( mAPrioriInfoGiven )
    insertAPrioriVote();

}

void StreetPatcher::setAPrioriVote( const ExtendedPose2d &pose, enumAPrioriStrength strength,
    bool inAllDirections )
{
  mAPrioriInfoGiven = true;
  mAPrioriPose = pose;
  mAPrioriVoteInAllDirections = inAllDirections;
  mAPrioriStrength = strength;
  LOGGING_INFO( lanedetectionLogger, "-------------------------" << endl);
  LOGGING_INFO( lanedetectionLogger, "Set a prioriVote" <<
                endl );
  if( inAllDirections )
  {
    LOGGING_INFO( lanedetectionLogger, "In ALL directions." << endl );
  }
  LOGGING_INFO( lanedetectionLogger, "-------------------------" << endl);
}

void StreetPatcher::insertAPrioriVote()
{
  cv::Point2f center( (mAPrioriPose.getX() - mCenterPose.getX() )*mMeters2Pixels + mCenter,
                      (mAPrioriPose.getY() - mCenterPose.getY() )*mMeters2Pixels + mCenter );

  cv::RotatedRect rRect( center, mCarVoteSize, 180.0f/M_PI*mAPrioriPose.getYaw() );
  cv::RotatedRect rRectThin( center, mCarVoteSizeThin, 180.0f/M_PI*mAPrioriPose.getYaw() );

  cv::Scalar voteLow;
  cv::Scalar voteHigh;
  if( mAPrioriStrength == STRENGTH_HIGH )
  {
    voteLow = cv::Scalar( mMinimumVoteForDetection - mMinimumVote*5 );
  } else if ( mAPrioriStrength == STRENGTH_MEDIUM ) {
    voteLow = cv::Scalar( mMinimumVoteForDetection - mMinimumVote*20 );
  } else if ( mAPrioriStrength == STRENGTH_LOW ) {
    voteLow = cv::Scalar( mMinimumVoteForDetection - mMinimumVote*40 );
  }
  voteHigh = cv::Scalar( mMinimumVote*3 );

  if( mAPrioriVoteInAllDirections )
  {
    for( int angleRegion = 0; angleRegion < mNumUsedAngleRegions; angleRegion ++ )
    {
      //cv::Scalar voteLow( mMinimumVoteForDetection - mMinimumVote*5 );
      //cv::Scalar voteHigh( mMinimumVote*3 );
      // VoteHigh is added additionally ontop of the lower vote.
      cv::Rect boundings = rRect.boundingRect();
      cv::Rect boundingsThin = rRectThin.boundingRect();
      addRectangle( mHoughSpace[angleRegion], rRect, boundings, voteLow );
      addRectangle( mHoughSpace[angleRegion], rRectThin, boundingsThin, voteHigh );
    }
  } else {
    int angleRegion = angle2AngleRegionID( mAPrioriPose.getYaw() );
    int angleRegion2 = getNeighbourAngleRegion( angleRegion, +1 );
    int angleRegion3 = getNeighbourAngleRegion( angleRegion, -1 );

    //cv::Scalar voteLow( mMinimumVoteForDetection - mMinimumVote*5 );
    //cv::Scalar voteHigh( mMinimumVote*3 );
    // VoteHigh is added additionally ontop of the lower vote.
    cv::Rect boundings = rRect.boundingRect();
    cv::Rect boundingsThin = rRectThin.boundingRect();
    addRectangle( mHoughSpace[angleRegion], rRect, boundings, voteLow );
    addRectangle( mHoughSpace[angleRegion], rRectThin, boundingsThin, voteHigh );
    addRectangle( mHoughSpace[angleRegion2], rRect, boundings, voteLow );
    addRectangle( mHoughSpace[angleRegion2], rRectThin, boundingsThin, voteHigh );
    addRectangle( mHoughSpace[angleRegion3], rRect, boundings, voteLow );
    addRectangle( mHoughSpace[angleRegion3], rRectThin, boundingsThin, voteHigh );
  }
}

void StreetPatcher::addFeatures( FeatureVector* features )
{
  FeatureVector::iterator it;
  for( it = mHistoricFeatures.begin(); it != mHistoricFeatures.end(); )
  {
    // Vote some features multiple times:
    // Find out if this feature is still used:
    if( (*it).frameCounter < (*it).maxNumberOfFrames )
    {
      (*it).frameCounter += 1;
      addFeature( (*it) );
      it++;
    }
    else
    {
      it = mHistoricFeatures.erase( it );
    }
  }

  for( it = features->begin(); it != features->end(); it ++ )
  {
    (*it).historicWeight = 1.0;
    (*it).frameCounter = 0;
    (*it).worldPose = mCoordConverter->car2World( mImgCarPose, (*it).pose );


    // Some feature-/vote-types should be voted for multiple frames.
    // If this is one of these, add it to the list of historic votes:
    if( mCarIsMoving )
    {
      unsigned int maximumHistoryLength = 0;	// max number of frames
      if( (*it).type == PARKING_LOT || (*it).type == PARKING_LINE  ||
          (*it).type == CROSSROAD_LINE ) {
        {
          // Calculate how long this feature should be kept in the history:
          for( unsigned int i = 0; i < mVotesForParkingLot[(*it).type].size(); i++ )
          {
            Vote vote = mVotesForParkingLot[(*it).type][i];
            unsigned int lengthForThisVote =
                vote.historySizeInMeters*mCurrentFramesPerMeter;
            //LOGGING_INFO( lanedetectionLogger, "\t\tlengthForThisVote: " << lengthForThisVote << endl );

            if( maximumHistoryLength < lengthForThisVote )
              maximumHistoryLength = lengthForThisVote;
          }
          // Calculate how long this feature should be kept in the history:
          for( unsigned int i = 0; i < mVotesForCrossSection[(*it).type].size(); i++ )
          {
            Vote vote = mVotesForCrossSection[(*it).type][i];
            unsigned int lengthForThisVote =
                vote.historySizeInMeters*mCurrentFramesPerMeter;

            if( maximumHistoryLength < lengthForThisVote )
              maximumHistoryLength = lengthForThisVote;
          }
          maximumHistoryLength = std::max( maximumHistoryLength, (unsigned int)1 );
          (*it).maxNumberOfFrames = maximumHistoryLength;
          // Normalize the weight of the vote to the number of Frames it should be used.
          // This "spreads out" the weight over multiple frames.
          (*it).historicWeight = 1.0/float(maximumHistoryLength);
          mHistoricFeatures.push_back( *it );

        }
      }
    }
    // Actually let the feature vote:
    addFeature( (*it) );
  }
  LOGGING_INFO( lanedetectionLogger, "Current historic features: "
                << mHistoricFeatures.size() << endl );
}

void StreetPatcher::addFeature( Feature &f )
{
  float patchAng = f.worldPose.getYaw();

  /*std::ofstream filestream( "/tmp/recordedData/Features/all.txt", std::ofstream::app );
          filestream << std::setprecision(20) << f.type << " " << f.probability << " " << f.pose;
          filestream.close();

          std::ofstream filestreamWorld( "/tmp/recordedData/Features/allWorld.txt", std::ofstream::app );
          filestreamWorld << std::setprecision(20) << f.type << " " << f.probability << " " << f.worldPose;
          filestreamWorld << "\t" << mImgCarPose;
          filestreamWorld.close();*/

  int xVote, yVote;
  float weight;

  for( unsigned int i = 0; i < mVotes[f.type].size(); i++ )
  {
    Vote vote = mVotes[f.type][i];
    float ang = patchAng + vote.angleOffset;

    // Determine which angle map this patch should vote into:
    int angleRegion = angle2AngleRegionID( ang );
    int angleRegionNext = getNeighbourAngleRegion( angleRegion, +1 );
    int angleRegionPrev = getNeighbourAngleRegion( angleRegion, -1 );

    // discretize the pose position and place it into the corresponding pixel:
    float y = (f.worldPose.getY() - mCenterPose.getY() )*mMeters2Pixels + mCenter;
    float x = (f.worldPose.getX() - mCenterPose.getX() )*mMeters2Pixels + mCenter;

    // Rotate vote according to patch angle, then add to Point (x,y):
    xVote = x + cos(patchAng)*vote.pos.x - sin(patchAng)*vote.pos.y;
    yVote = y + sin(patchAng)*vote.pos.x + cos(patchAng)*vote.pos.y;

    cv::RotatedRect rr( cv::Point2f( xVote, yVote ), vote.size, patchAng*180.0/M_PI );
    cv::Rect boundings = rr.boundingRect();

    addRectangle( mHoughSpace[angleRegion], rr, boundings, vote.weight );
    addRectangle( mHoughSpace[angleRegionNext], rr, boundings, vote.weight*0.5 );
    addRectangle( mHoughSpace[angleRegionPrev], rr, boundings, vote.weight*0.5 );
  }

  if( mVotesForCrossSection[f.type].size() > 0 || mVotesForParkingLot[f.type].size() > 0 )
  {
    for( unsigned int i = 0; i < mVotesForCrossSection[f.type].size(); i++ )
    {
      Vote vote = mVotesForCrossSection[f.type][i];

      float ang = patchAng + vote.angleOffset;

      // Determine which angle map this patch should vote into:
      int angleRegion = angle2AngleRegionID( ang );
      int angleRegionNext = getNeighbourAngleRegion( angleRegion, +1 );
      int angleRegionPrev = getNeighbourAngleRegion( angleRegion, -1 );

      // discretize the pose position and place it into the corresponding pixel:
      float y = (f.worldPose.getY() - mCenterPose.getY() )*mMeters2Pixels + mCenter;
      float x = (f.worldPose.getX() - mCenterPose.getX() )*mMeters2Pixels + mCenter;

      // Rotate vote according to patch angle, then add to Point (x,y):
      xVote = x + cos(patchAng)*vote.pos.x - sin(patchAng)*vote.pos.y;
      yVote = y + sin(patchAng)*vote.pos.x + cos(patchAng)*vote.pos.y;

      cv::RotatedRect rr( cv::Point2f( xVote, yVote ), vote.size, patchAng*180.0/M_PI );
      cv::Rect boundings = rr.boundingRect();

      weight = vote.weight*f.historicWeight;
      if( mBoostCrossSectionFeatures )
      {
        weight = weight*2.8;    // Boost the features.
      }

      addRectangle( mHoughSpaceForCrossSection[angleRegion],
                    rr, boundings, weight );
      addRectangle( mHoughSpaceForCrossSection[angleRegionNext],
                    rr, boundings, weight );
      addRectangle( mHoughSpaceForCrossSection[angleRegionPrev],
                    rr, boundings, weight );

    }

    for( unsigned int i = 0; i < mVotesForParkingLot[f.type].size(); i++ )
    {
      Vote vote = mVotesForParkingLot[f.type][i];

      float ang = patchAng + vote.angleOffset;

      // Determine which angle map this patch should vote into:
      int angleRegion = angle2AngleRegionID( ang );
      /*int angleRegionNext = getNeighbourAngleRegion( angleRegion, +1 );
                          int angleRegionPrev = getNeighbourAngleRegion( angleRegion, -1 );*/

      // discretize the pose position and place it into the corresponding pixel:
      float y = (f.worldPose.getY() - mCenterPose.getY() )*mMeters2Pixels + mCenter;
      float x = (f.worldPose.getX() - mCenterPose.getX() )*mMeters2Pixels + mCenter;

      // Rotate vote according to patch angle, then add to Point (x,y):
      xVote = x + cos(patchAng)*vote.pos.x - sin(patchAng)*vote.pos.y;
      yVote = y + sin(patchAng)*vote.pos.x + cos(patchAng)*vote.pos.y;

      cv::RotatedRect rr( cv::Point2f( xVote, yVote ), vote.size, patchAng*180.0/M_PI );
      cv::Rect boundings = rr.boundingRect();

      weight = vote.weight*f.historicWeight;

      addRectangle( mHoughSpaceForParkingLot[angleRegion][vote.separationID],
                    rr, boundings, weight );
      /*addRectangle( mHoughSpaceForParkingLot[angleRegionNext],
                          rr, boundings, vote.weight );
                          addRectangle( mHoughSpaceForParkingLot[angleRegionPrev],
                          rr, boundings, vote.weight );*/

    }
  }

}

void StreetPatcher::addPreviousPatch( PatchPtr patch, bool voteForSpecialPatches )
{
  /*float ang = patch->getPose().getYaw();
        // Determine which angle map this patch should vote into:
        int angleRegion = angle2AngleRegionID( ang );
        int angleRegionNext = getNeighbourAngleRegion( angleRegion, +1 );
        int angleRegionPrev = getNeighbourAngleRegion( angleRegion, -1 );

        float x = (patch->getPose().getX()-mCenterPose.getX())*mMeters2Pixels + mCenter;
        float y = (patch->getPose().getY()-mCenterPose.getY())*mMeters2Pixels + mCenter;
        float vote = std::min( patch->getProbability()*25, 1.0 )*(mMinimumVoteForDetection - 1);

        cv::RotatedRect rr( cv::Point2f( x, y ),
        cv::Size( 0.1*mMeters2Pixels, 0.1*mMeters2Pixels ),
        ang*180/M_PI );

        if( patch->getPatchType() == STRAIGHT )
        {
        addRectangle( mHoughSpace[angleRegion], rr, vote );
        addRectangle( mHoughSpace[angleRegionNext], rr, vote*0.75 );
        addRectangle( mHoughSpace[angleRegionPrev], rr, vote*0.75 );
        } else if( patch->getPatchType() == CROSS_SECTION ) {
        addRectangle( mHoughSpaceForCrossSection[angleRegion], rr, vote );
        addRectangle( mHoughSpaceForCrossSection[angleRegionNext], rr, vote*0.75 );
        addRectangle( mHoughSpaceForCrossSection[angleRegionPrev], rr, vote*0.75 );
        }*/

  insertPatchVote( patch, voteForSpecialPatches );
}

void StreetPatcher::findPatches( unsigned int maximumNum )
{
  double min;
  cv::Point min_loc;
  double max[mNumUsedAngleRegions];
  double maxC[mNumUsedAngleRegions];	// for cross sections
  double maxP[mNumUsedAngleRegions];	// for cross sections
  cv::Point max_loc[mNumUsedAngleRegions];
  cv::Point maxC_loc[mNumUsedAngleRegions];	// for cross sections
  cv::Point maxP_loc[mNumUsedAngleRegions];	// for cross sections
  cv::Mat tmp;
  cv::Size blurSize( mBlurSize, mBlurSize );

  // for every orientation, find the maximum:
  for( unsigned int i = 0; i < mNumUsedAngleRegions; i++ )
  {
    //cv::GaussianBlur(mHoughSpace[i], mHoughSpace[i], blurSize, 0, 0 );
    cv::blur( mHoughSpace[i], mHoughSpace[i], blurSize );
    cv::blur( mHoughSpaceForCrossSection[i], mHoughSpaceForCrossSection[i], blurSize );
    cv::blur( mHoughSpaceForParkingLot[i][0], mHoughSpaceForParkingLot[i][0], blurSize );
    cv::blur( mHoughSpaceForParkingLot[i][1], mHoughSpaceForParkingLot[i][1], blurSize );
    //mHoughSpace[i] = tmp;
    cv::minMaxLoc(mHoughSpace[i], &min, &max[i], &min_loc, &max_loc[i]);
    cv::minMaxLoc(mHoughSpaceForCrossSection[i], &min, &maxC[i], &min_loc, &maxC_loc[i]);

    mCombinedParkingHoughSpace = cv::min( mHoughSpaceForParkingLot[i][0],
        mHoughSpaceForParkingLot[i][1] );
    cv::minMaxLoc( mCombinedParkingHoughSpace, &min, &maxP[i], &min_loc, &maxP_loc[i]);

    if( mDebugMode )
    {
      mHoughSpace[i].copyTo( mDebugHoughSpace( cv::Rect( mCols*i, mRows*3, mCols, mRows ) ) );
      mHoughSpaceForCrossSection[i].copyTo( mDebugHoughSpace( cv::Rect( mCols*i, mRows*2, mCols, mRows ) ) );
      mHoughSpaceForParkingLot[i][0].copyTo( mDebugHoughSpace( cv::Rect( mCols*i, mRows, mCols, mRows ) ) );
      mHoughSpaceForParkingLot[i][1].copyTo( mDebugHoughSpace( cv::Rect( mCols*i, 0, mCols, mRows ) ) );
    }
  }

  int absMax = 0;
  int maxRegion = 0;
  for( size_t patchNum = 0; patchNum < maximumNum; patchNum ++ )
  {
    absMax = 0;
    // Find out in which region the maximum was found:
    for( unsigned int i = 0; i < mNumUsedAngleRegions; i++ )
    {
      if( max[i] > absMax )
      {
        absMax = max[i];
        maxRegion = i;
      }
    }
    LOGGING_INFO( lanedetectionLogger, "Maximum detected for STRAIGHT: " << absMax << endl );

    // If at least one region was found where the highest vote is higher than required min:
    if( absMax > mMinimumVoteForDetection )
    {
      PatchPtr newPatch( new Patch( STRAIGHT, ExtendedPose2d(
                                      mCenterPose.getX() + (max_loc[maxRegion].x-mCenter)/mMeters2Pixels,
                                      mCenterPose.getY() + (max_loc[maxRegion].y-mCenter)/mMeters2Pixels,
                                      angleRegion2Angle( maxRegion ) ) ) );

      newPatch->setProbability( float(absMax)/float(65536) );

      mPatches.push_back( newPatch );
      //insertPatchVote( newPatch, false );

      // Draw black over votes (so they won't be used again):
      //int angleRegionNext = getNeighbourAngleRegion( maxRegion, +1 );
      //int angleRegionPrev = getNeighbourAngleRegion( maxRegion, -1 );
      setRegion( maxRegion, max_loc[maxRegion], mClearRegionSize );
      //setRegion( angleRegionNext, max_loc[maxRegion], mClearRegionSize );
      //setRegion( angleRegionPrev, max_loc[maxRegion], mClearRegionSize );

      // Find new maximum:
      cv::minMaxLoc(mHoughSpace[maxRegion], &min, &max[maxRegion], &min_loc, &max_loc[maxRegion]);
      //cv::minMaxLoc(mHoughSpace[angleRegionNext], &min, &max[angleRegionNext], &min_loc, &max_loc[angleRegionNext]);
      //cv::minMaxLoc(mHoughSpace[angleRegionPrev], &min, &max[angleRegionPrev], &min_loc, &max_loc[angleRegionPrev]);
    } else {
      break;
    }
  }

  // Find cross sections:
  // There should never be more than one cross section, so only
  // allow one maximum to be found:
  if( mSearchForCrossSections )
  {
    absMax = 0;
    // Find out in which region the maximum was found:
    for( unsigned int i = 0; i < mNumUsedAngleRegions; i++ )
    {
      if( maxC[i] > absMax )
      {
        absMax = maxC[i];
        maxRegion = i;
      }
    }

    LOGGING_INFO( lanedetectionLogger, "Maximum detected for CROSS_SECTION: " << absMax << endl );

    // If at least one region was found where the highest vote is higher than required min:
    if( absMax > mMinimumVoteForDetectionCrossSection )
    {
      PatchPtr newPatch( new Patch( CROSS_SECTION, ExtendedPose2d(
                                      mCenterPose.getX() + (maxC_loc[maxRegion].x-mCenter)/mMeters2Pixels,
                                      mCenterPose.getY() + (maxC_loc[maxRegion].y-mCenter)/mMeters2Pixels,
                                      angleRegion2Angle(maxRegion) ) ) );
      newPatch->setProbability( float(absMax)/float(65536) );
      mPatches.push_back( newPatch );
      //insertPatchVote( newPatch );
    }
  }

  // Find parking lots:
  if( mSearchForParkingLots )
  {
    absMax = 0;
    // Find out in which region the maximum was found:
    for( unsigned int i = 0; i < mNumUsedAngleRegions; i++ )
    {
      if( maxP[i] > absMax )
      {
        absMax = maxP[i];
        maxRegion = i;
      }
    }

    LOGGING_INFO( lanedetectionLogger, "Maximum detected for PARKING: " << absMax << endl );

    // If at least one region was found where the highest vote is higher than required min:
    if( absMax > mMinimumVoteForDetectionParkingLot )
    {
      PatchPtr newPatch( new Patch( PARKING, ExtendedPose2d(
                                      mCenterPose.getX() + (maxP_loc[maxRegion].x-mCenter)/mMeters2Pixels,
                                      mCenterPose.getY() + (maxP_loc[maxRegion].y-mCenter)/mMeters2Pixels,
                                      angleRegion2Angle(maxRegion) ) ) );
      newPatch->setProbability( float(absMax)/float(65536) );
      mPatches.push_back( newPatch );
      //insertPatchVote( newPatch );
    }
  }

}

void StreetPatcher::setRegion( int angleRegion, cv::Point p, float size, int newAmount )
{
  cv::Rect r( p - cv::Point( size*0.5, size*0.5 ),
              cv::Size( size, size ) );
  cv::rectangle( mHoughSpace[angleRegion], r, cv::Scalar( newAmount ), CV_FILLED );
  //cv::rectangle( mHoughSpaceForCrossSection[angleRegion], r, cv::Scalar( newAmount ), CV_FILLED );
}

void StreetPatcher::addRectangle( cv::Mat &mat, const cv::RotatedRect &rRect,
                                  const cv::Rect &boundings, cv::Scalar amount )
{
  // Make sure we're inside the image:
  cv::Point2f tl = boundings.tl();
  if( tl.x < 0 || tl.y < 0 ) return;
  cv::Point2f br = boundings.br();
  if( br.x >= mat.size().width || br.y >= mat.size().height ) return;

  cv::Mat roi = mat(boundings);

  cv::Mat tmp( boundings.size(), CV_16UC1, cv::Scalar(0) );

  cv::Point2f vertices[4];
  rRect.points(vertices);
  cv::Point2i verticesI[4];
  cv::Point2f center( roi.size().width*0.5, roi.size().height*0.5 );
  for( unsigned int i = 0; i < 4; i++ )
    verticesI[i] = vertices[i] - rRect.center + center;

  cv::fillConvexPoly( tmp, &verticesI[0], 4, amount );

  cv::add( roi, tmp, roi );
}

void StreetPatcher::insertPatchVote( PatchPtr patch, bool voteForSpecialPatches )
{
  for( unsigned int i = 0; i < mPatchVotes[patch->getPatchType()].size(); i++ )
  {
    Vote vote = mPatchVotes[patch->getPatchType()][i];

    float ang = patch->getPose().getYaw() + vote.angleOffset;
    // Determine which angle map this patch should vote into:
    int angleRegion = angle2AngleRegionID( ang );
    //int angleRegionNext = getNeighbourAngleRegion( angleRegion, +1 );
    //int angleRegionPrev = getNeighbourAngleRegion( angleRegion, -1 );

    // discretize the pose position and place it into the corresponding pixel:
    float y = (patch->getPose().getY() - mCenterPose.getY() )*mMeters2Pixels + mCenter;
    float x = (patch->getPose().getX() - mCenterPose.getX() )*mMeters2Pixels + mCenter;

    //xVote = x + mPatchVotes[patch.type][i].pos.x*cos(patch.pose.getYaw());
    //yVote = y + mPatchVotes[patch.type][i].pos.y*sin(patch.pose.getYaw());
    // Rotate vote according to patch angle, then add to Point (x,y):
    int xVote, yVote;
    xVote = x + cos(ang)*vote.pos.x - sin(ang)*vote.pos.y;
    yVote = y + sin(ang)*vote.pos.x + cos(ang)*vote.pos.y;

    int voteAmount = vote.weight;//*std::min( patch->getProbability()*50, 1.0 );

    cv::RotatedRect rr( cv::Point2f( xVote, yVote ), vote.size, ang*180.0/M_PI );
    cv::Rect boundings = rr.boundingRect();

    addRectangle( mHoughSpace[angleRegion], rr, boundings, voteAmount );
    //addRectangle( mHoughSpace[angleRegionNext], rr, voteAmount );
    //addRectangle( mHoughSpace[angleRegionPrev], rr, voteAmount );

    //mHoughSpace[angleRegion].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegion, 2*mRows, mCols, mRows ) ) );
    //mHoughSpace[angleRegionNext].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegionNext, mRows, mCols, mRows ) ) );
    //mHoughSpace[angleRegionPrev].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegionPrev, mRows, mCols, mRows ) ) );
  }

  if( voteForSpecialPatches )
  {
    if( mSearchForCrossSections )
    {
      for( unsigned int i = 0; i < mPatchVotesForCrossSection[patch->getPatchType()].size(); i++ )
      {
        Vote vote = mPatchVotesForCrossSection[patch->getPatchType()][i];

        float ang = patch->getPose().getYaw() + vote.angleOffset;
        // Determine which angle map this patch should vote into:
        int angleRegion = angle2AngleRegionID( ang );
        //int angleRegionNext = getNeighbourAngleRegion( angleRegion, +1 );
        //int angleRegionPrev = getNeighbourAngleRegion( angleRegion, -1 );

        // discretize the pose position and place it into the corresponding pixel:
        float y = (patch->getPose().getY() - mCenterPose.getY() )*mMeters2Pixels + mCenter;
        float x = (patch->getPose().getX() - mCenterPose.getX() )*mMeters2Pixels + mCenter;

        //xVote = x + mPatchVotes[patch.type][i].pos.x*cos(patch.pose.getYaw());
        //yVote = y + mPatchVotes[patch.type][i].pos.y*sin(patch.pose.getYaw());
        // Rotate vote according to patch angle, then add to Point (x,y):
        int xVote, yVote;
        xVote = x + cos(ang)*vote.pos.x - sin(ang)*vote.pos.y;
        yVote = y + sin(ang)*vote.pos.x + cos(ang)*vote.pos.y;

        int voteAmount = vote.weight;//*std::min( patch->getProbability()*50, 1.0 );

        cv::RotatedRect rr( cv::Point2f( xVote, yVote ), vote.size, ang*180.0/M_PI );
        cv::Rect boundings = rr.boundingRect();

        addRectangle( mHoughSpaceForCrossSection[angleRegion], rr, boundings, voteAmount );
        //addRectangle( mHoughSpaceForCrossSection[angleRegionNext], rr, voteAmount );
        //addRectangle( mHoughSpaceForCrossSection[angleRegionPrev], rr, voteAmount );

        //mHoughSpaceForCrossSection[angleRegion].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegion, mRows, mCols, mRows ) ) );
        //mHoughSpaceForCrossSection[angleRegionNext].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegionNext, 0, mCols, mRows ) ) );
        //mHoughSpaceForCrossSection[angleRegionPrev].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegionPrev, 0, mCols, mRows ) ) );
      }
    }

    if( mSearchForParkingLots )
    {
      for( unsigned int i = 0; i < mPatchVotesForParkingLot[patch->getPatchType()].size(); i++ )
      {
        Vote vote = mPatchVotesForParkingLot[patch->getPatchType()][i];

        float ang = patch->getPose().getYaw() + vote.angleOffset;
        // Determine which angle map this patch should vote into:
        int angleRegion = angle2AngleRegionID( ang );
        //int angleRegionNext = getNeighbourAngleRegion( angleRegion, +1 );
        //int angleRegionPrev = getNeighbourAngleRegion( angleRegion, -1 );

        // discretize the pose position and place it into the corresponding pixel:
        float y = (patch->getPose().getY() - mCenterPose.getY() )*mMeters2Pixels + mCenter;
        float x = (patch->getPose().getX() - mCenterPose.getX() )*mMeters2Pixels + mCenter;

        //xVote = x + mPatchVotes[patch.type][i].pos.x*cos(patch.pose.getYaw());
        //yVote = y + mPatchVotes[patch.type][i].pos.y*sin(patch.pose.getYaw());
        // Rotate vote according to patch angle, then add to Point (x,y):
        int xVote, yVote;
        xVote = x + cos(ang)*vote.pos.x - sin(ang)*vote.pos.y;
        yVote = y + sin(ang)*vote.pos.x + cos(ang)*vote.pos.y;

        //int voteAmount = vote.weight*std::min( patch->getProbability()*50, 1.0 );
        int voteAmount = vote.weight;

        cv::RotatedRect rr( cv::Point2f( xVote, yVote ), vote.size, ang*180.0/M_PI );
        cv::Rect boundings = rr.boundingRect();

        addRectangle( mHoughSpaceForParkingLot[angleRegion][0],
            rr, boundings, voteAmount );
        addRectangle( mHoughSpaceForParkingLot[angleRegion][1],
            rr, boundings, voteAmount );
        //addRectangle( mHoughSpaceForParkingLot[angleRegionNext], rr, voteAmount );
        //addRectangle( mHoughSpaceForParkingLot[angleRegionPrev], rr, voteAmount );

        //mHoughSpaceForParkingLot[angleRegion][0].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegion, mRows, mCols, mRows ) ) );
        //mHoughSpaceForParkingLot[angleRegion][1].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegion, 0, mCols, mRows ) ) );

        //mHoughSpaceForParkingLot[angleRegionNext].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegionNext, 0, mCols, mRows ) ) );
        //mHoughSpaceForParkingLot[angleRegionPrev].copyTo( mDebugHoughSpace( cv::Rect( mCols*angleRegionPrev, 0, mCols, mRows ) ) );
      }
    }

  }

}

void StreetPatcher::clearFeatures()
{
  mHoughSpace.clear();
  mHoughSpaceForCrossSection.clear();
  mHoughSpaceForParkingLot.clear();
  for( unsigned short i =  0; i < mNumUsedAngleRegions; i++ )
  {
    mHoughSpace.push_back( cv::Mat( mRows, mCols, CV_16UC1, cv::Scalar(0)) );
    mHoughSpaceForCrossSection.push_back( cv::Mat( mRows, mCols, CV_16UC1, cv::Scalar(0)) );
    // 4D for Parking lots, because we need to separate votes going forward and backwards.
    std::vector< cv::Mat > tmp;
    tmp.push_back( cv::Mat( mRows, mCols, CV_16UC1, cv::Scalar(0)) );
    tmp.push_back( cv::Mat( mRows, mCols, CV_16UC1, cv::Scalar(0)) );
    mHoughSpaceForParkingLot.push_back( tmp );
  }
  mHoughSpaceForTrafficSigns = cv::Mat( mRows, mCols, CV_16UC1, cv::Scalar(0) );
  mCombinedParkingHoughSpace = cv::Mat( mRows, mCols, CV_16UC1, cv::Scalar(0) );
}


void StreetPatcher::initFeatureVotes()
{
  Vote vote;

  ////////////////////////////////////////////
  // SIDE_LINE:
  //      ||              ||
  //      ||      |       ||
  //      ||      |<------||----->
  //      ||      |       ||
  //      ||              ||
  //

  int voteWidth = 0.05*mMeters2Pixels;
  int voteLength = 0.5*mMeters2Pixels;
  vote.historySizeInMeters = 0;
  vote.pos.y = STREET_SIDE_TO_CENTER*mMeters2Pixels;
  vote.pos.x = 0;
  vote.size.width = voteLength;
  vote.size.height = voteWidth;
  vote.weight = mMinimumVote*2;
  vote.angleOffset = 0;
  mVotes[SIDE_LINE].push_back( vote );

  vote.pos.y = -STREET_SIDE_TO_CENTER*mMeters2Pixels;
  vote.pos.x = 0;
  vote.size.width = voteLength;
  vote.size.height = voteWidth;
  vote.weight = mMinimumVote*2;
  vote.angleOffset = 0;
  mVotes[SIDE_LINE].push_back( vote );

  // Because there are a lot of miss-clasifications of center lines
  // as being side lines, also let the side line vote as a center line:
  if( mLetSideVoteAsCenter )
  {
    vote.pos.y = 0;
    vote.pos.x = 0;
    vote.size.width = voteLength;
    vote.size.height = voteWidth;
    vote.weight = mMinimumVote*mFeatureVoteCoupling*2;
    vote.angleOffset = 0;
    mVotes[SIDE_LINE].push_back( vote );
  }

  // Side line also slightly votes for cross section:
  /*voteWidth = 0.05*mMeters2Pixels;
  voteLength = 1.0*mMeters2Pixels;
  vote.pos.y = -STREET_SIDE_TO_CENTER*mMeters2Pixels;
  vote.pos.x = -STREET_SIDE_TO_CENTER*mMeters2Pixels;
  vote.size.width = voteLength;
  vote.size.height = voteWidth;
  vote.weight = mMinimumVote;
  vote.angleOffset = M_PI_2;
  mVotesForCrossSection[SIDE_LINE].push_back( vote );
  vote.pos.y = -STREET_SIDE_TO_CENTER*mMeters2Pixels;
  vote.pos.x = STREET_SIDE_TO_CENTER*mMeters2Pixels;
  mVotesForCrossSection[SIDE_LINE].push_back( vote );
  vote.pos.y = STREET_SIDE_TO_CENTER*mMeters2Pixels;
  vote.pos.x = STREET_SIDE_TO_CENTER*mMeters2Pixels;
  mVotesForCrossSection[SIDE_LINE].push_back( vote );
  vote.pos.y = STREET_SIDE_TO_CENTER*mMeters2Pixels;
  vote.pos.x = -STREET_SIDE_TO_CENTER*mMeters2Pixels;
  mVotesForCrossSection[SIDE_LINE].push_back( vote );*/

  ////////////////////////////////////////////
  // CENTER_LINE:
  //      ||              ||
  //      ||      |       ||
  //      ||     >|<      ||
  //      ||      |       ||
  //      ||              ||

  vote.pos.y = 0;
  vote.pos.x = 0;
  vote.size.width = voteLength;
  vote.size.height = voteWidth;
  vote.weight = mMinimumVote*2;
  vote.angleOffset = 0;
  mVotes[CENTER_LINE].push_back( vote );

  // Also let Center line vote as if it was a side line,
  // because there are many falsly-detected center-line features
  // on the side lines (mainly at 45°. TODO: Bug? Let Jan double-
  // check!)
  if( mLetCenterVoteAsSide )
  {
    vote.pos.y = STREET_SIDE_TO_CENTER*mMeters2Pixels;
    vote.pos.x = 0;
    vote.size.width = voteLength;
    vote.size.height = voteWidth;
    vote.weight = mMinimumVote*mFeatureVoteCoupling*2;
    vote.angleOffset = 0;
    mVotes[CENTER_LINE].push_back( vote );

    vote.pos.y = -STREET_SIDE_TO_CENTER*mMeters2Pixels;
    vote.pos.x = 0;
    vote.size.width = voteLength;
    vote.size.height = voteWidth;
    vote.weight = mMinimumVote*mFeatureVoteCoupling*2;
    vote.angleOffset = 0;
    mVotes[CENTER_LINE].push_back( vote );
  }

  ////////////////////////////////////////////
  // CROSSROAD_LINE:
  //
  //			<---
  //				|
  //				|
  // ||		----x---||
  // ||		|		||
  // ||		|		||
  // Features vote for center of cross-road patch.
  voteWidth = 0.3*mMeters2Pixels;
  voteLength = 0.03*mMeters2Pixels;
  vote.size.width = voteWidth;
  vote.weight = mMinimumVote*2.5;
  vote.historySizeInMeters = PATCH_LENGTHS[CROSS_SECTION]*0.75;
  // Insert for every 90 degrees angle-offset:
  for( float angle = 0; angle < 2*M_PI; angle += M_PI_2 )
  {
    vote.angleOffset = angle;

    vote.pos.x = -(LANE_WIDTH*0.5)*mMeters2Pixels;
    vote.pos.y = CROSSROAD_LINE_TO_CROSS_SECTION*mMeters2Pixels;
    mVotesForCrossSection[CROSSROAD_LINE].push_back( vote );

    // Also vote in other direction:
    // (For crossroad lines detected on the other side of the
    // cross section, which should vote "backwards" towards the
    // car)
    vote.pos.x = (LANE_WIDTH*0.5)*mMeters2Pixels;
    vote.pos.y = -CROSSROAD_LINE_TO_CROSS_SECTION*mMeters2Pixels;
    mVotesForCrossSection[CROSSROAD_LINE].push_back( vote );

  }

  ////////////////////////////////////////////
  // CROSSROAD_CORNER:
  voteWidth = 0.03*mMeters2Pixels;
  voteLength = 0.03*mMeters2Pixels;
  vote.historySizeInMeters = PATCH_LENGTHS[CROSS_SECTION]*0.75;
  vote.size.height = voteLength;
  vote.size.width = voteWidth;
  vote.weight = mMinimumVote*2;
  vote.angleOffset = 0;
  vote.pos.x = (LANE_WIDTH)*mMeters2Pixels;
  vote.pos.y = LANE_WIDTH*mMeters2Pixels;
  mVotesForCrossSection[CROSSROAD_CORNER].push_back( vote );

  ////////////////////////////////////////////
  // PARKING:
  // Let T-Feature and Line-Feature vote for parking lots:
  // Parallel parking lots:
  // T-Feature
  vote.historySizeInMeters = PATCH_LENGTHS[PARKING]*3;
  /*vote.pos.x = PATCH_LENGTHS[PARKING]*0.5*mMeters2Pixels;
  vote.pos.y = -PATCH_WIDTHS[PARKING]*0.5*mMeters2Pixels;
  vote.size.width = 0.02*mMeters2Pixels;
  vote.size.height = 0.3*mMeters2Pixels;
  vote.weight = mMinimumVote*10;
  vote.angleOffset = 0;
  vote.separationID = 0;
  mVotesForParkingLot[PARKING_LOT].push_back( vote );
  vote.separationID = 1;
  vote.pos.x = -PATCH_LENGTHS[PARKING]*0.5*mMeters2Pixels;
  mVotesForParkingLot[PARKING_LOT].push_back( vote );*/

  // Line-Feature
  vote.angleOffset = -M_PI*0.5;
  vote.size.width = PATCH_WIDTHS[PARKING]*mMeters2Pixels;
  vote.pos.y = PATCH_LENGTHS[PARKING]*0.5*mMeters2Pixels;
  vote.pos.x = -PATCH_WIDTHS[PARKING]*0.25*mMeters2Pixels;
  vote.size.height = 0.02*mMeters2Pixels;
  vote.weight = mMinimumVote*30;
  vote.separationID = 0;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = -M_PI*0.5 + mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = -M_PI*0.5 - mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.separationID = 1;
  vote.angleOffset = -M_PI*0.5;
  vote.pos.y = -PATCH_LENGTHS[PARKING]*0.5*mMeters2Pixels;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = -M_PI*0.5 + mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = -M_PI*0.5 - mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );

  // Cross parking lots:
  // T-Feature
  vote.historySizeInMeters = PATCH_WIDTHS[PARKING]*2;
  /*vote.angleOffset = M_PI*0.5;
  vote.pos.y = -PATCH_LENGTHS[PARKING]*0.5*mMeters2Pixels;
  vote.pos.x = PATCH_WIDTHS[PARKING]*0.5*mMeters2Pixels;
  vote.size.width = 0.02*mMeters2Pixels;
  vote.size.height = 0.3*mMeters2Pixels;
  vote.weight = mMinimumVote*6;
  vote.separationID = 0;
  mVotesForParkingLot[PARKING_LOT].push_back( vote );
  vote.separationID = 1;
  vote.pos.x = -PATCH_WIDTHS[PARKING]*0.5*mMeters2Pixels;
  mVotesForParkingLot[PARKING_LOT].push_back( vote );*/

  // Line-Feature
  vote.angleOffset = 0;
  vote.size.width = PATCH_WIDTHS[PARKING]*mMeters2Pixels;
  vote.pos.y = PATCH_WIDTHS[PARKING]*0.5*mMeters2Pixels;
  vote.pos.x = -PATCH_LENGTHS[PARKING]*0.25*mMeters2Pixels;
  vote.size.height = 0.01*mMeters2Pixels;
  vote.weight = mMinimumVote*30;
  vote.separationID = 0;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.separationID = 1;
  vote.angleOffset = 0;
  vote.pos.y = -PATCH_WIDTHS[PARKING]*0.5*mMeters2Pixels;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  /*vote.angleOffset = 0;
  vote.size.width = PATCH_WIDTHS[PARKING]*mMeters2Pixels;
  vote.pos.y = PATCH_WIDTHS[PARKING]*0.5*mMeters2Pixels;
  vote.pos.x = -PATCH_LENGTHS[PARKING]*0.5*mMeters2Pixels;
  vote.size.height = 0.02*mMeters2Pixels;
  vote.weight = mMinimumVote*50;
  vote.separationID = 0;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mVotesForParkingLot[PARKING_LINE].push_back( vote );*/
}

void StreetPatcher::initPatchVotes()
{
  Vote vote;

  ////////////////////////////////////////////
  // Let STRAIGHT patches vote for other STRAIGHT patches:
  int width = 0.55*mMeters2Pixels;
  //int length = 0.15*mMeters2Pixels;
  int length = 0.1*mMeters2Pixels;
  vote.pos.x = PATCH_LENGTHS[STRAIGHT]*mMeters2Pixels;
  vote.pos.y = 0;
  vote.size.height = width;
  vote.size.width = length;
  vote.weight = mMinimumVoteForDetection-3*mMinimumVote;
  vote.angleOffset = 0;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = 2*mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.angleOffset = -2*mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = 0;
  vote.pos.x = 2*PATCH_LENGTHS[STRAIGHT]*mMeters2Pixels;
  mPatchVotes[STRAIGHT].push_back( vote );

  vote.pos.x = -PATCH_LENGTHS[STRAIGHT]*mMeters2Pixels;
  vote.pos.y = 0;
  vote.size.height = width;
  vote.size.width = length;
  vote.weight = mMinimumVoteForDetection-3*mMinimumVote;
  vote.angleOffset = 0;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = 2*mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.angleOffset = -2*mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = 0;
  vote.pos.x = -2*PATCH_LENGTHS[STRAIGHT]*mMeters2Pixels;
  mPatchVotes[STRAIGHT].push_back( vote );

  vote.pos.x = PATCH_LENGTHS[STRAIGHT]*mMeters2Pixels;
  vote.pos.y = 0;
  vote.size.height = width;
  vote.size.width = length;
  vote.weight = mMinimumVoteForDetection-3*mMinimumVote;
  vote.angleOffset = M_PI;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = M_PI + mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.angleOffset = M_PI - mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = M_PI + 2*mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.angleOffset = M_PI - 2*mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = M_PI;
  vote.pos.x = 2*PATCH_LENGTHS[STRAIGHT]*mMeters2Pixels;
  mPatchVotes[STRAIGHT].push_back( vote );

  vote.pos.x = -PATCH_LENGTHS[STRAIGHT]*mMeters2Pixels;
  vote.pos.y = 0;
  vote.size.height = width;
  vote.size.width = length;
  vote.weight = mMinimumVoteForDetection-3*mMinimumVote;
  vote.angleOffset = M_PI;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = M_PI + mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.angleOffset = M_PI - mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = M_PI + 2*mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.angleOffset = M_PI - 2*mAngleRegionSize;
  mPatchVotes[STRAIGHT].push_back( vote );
  vote.weight = mMinimumVoteForDetection-4*mMinimumVote;
  vote.angleOffset = M_PI;
  vote.pos.x = -2*PATCH_LENGTHS[STRAIGHT]*mMeters2Pixels;
  mPatchVotes[STRAIGHT].push_back( vote );


  ////////////////////////////////////////////
  // Let STRAIGHT patches vote for CROSS_SECTION patches:
  width = 0.1*mMeters2Pixels;
  length = 0.75*mMeters2Pixels;

  vote.pos.x = 0.75*mMeters2Pixels;
  vote.pos.y = 0;
  vote.size.height = width;
  vote.size.width = length;
  vote.weight = mMinimumVoteForDetectionCrossSection;
  vote.angleOffset = 0;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );

  /*vote.weight = mMinimumVoteForDetectionCrossSection - 15*mMinimumVote;
  vote.angleOffset = mAngleRegionSize;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );*/

  vote.pos.x = -0.75*mMeters2Pixels;
  vote.pos.y = 0;
  vote.size.height = width;
  vote.size.width = length;
  vote.weight = mMinimumVoteForDetectionCrossSection;
  vote.angleOffset = 0;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );

  /*vote.weight = mMinimumVoteForDetectionCrossSection - 15*mMinimumVote;
  vote.angleOffset = mAngleRegionSize;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );*/

  vote.pos.y = 0.75*mMeters2Pixels;
  vote.pos.x = 0;
  vote.size.height = length;
  vote.size.width = width;
  vote.weight = mMinimumVoteForDetectionCrossSection;
  vote.angleOffset = M_PI_2;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );

  /*vote.weight = mMinimumVoteForDetectionCrossSection - 15*mMinimumVote;
  vote.angleOffset = mAngleRegionSize;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );*/

  vote.pos.y = -0.75*mMeters2Pixels;
  vote.pos.x = 0;
  vote.size.height = length;
  vote.size.width = width;
  vote.weight = mMinimumVoteForDetectionCrossSection;
  vote.angleOffset = M_PI_2;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );

  /*vote.weight = mMinimumVoteForDetectionCrossSection - 15*mMinimumVote;
  vote.angleOffset = mAngleRegionSize;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mPatchVotesForCrossSection[STRAIGHT].push_back( vote );*/


  /*width = 0.1*mMeters2Pixels;
        length = 2.0*mMeters2Pixels;
        vote.pos.x = 0;
        vote.pos.y = PATCH_LENGTHS[STRAIGHT];
        vote.size.height = width;
        vote.size.width = length;
        vote.weight = mMinimumVoteForDetectionCrossSection-1;
        vote.angleOffset = 0;
        mPatchVotesForCrossSection[STRAIGHT].push_back( vote );*/

  ////////////////////////////////////////////
  // Let STRAIGHT patches vote for "parallel" PARKING patches:
  width = 0.02*mMeters2Pixels;
  length = PATCH_LENGTHS_PARKING_PARALLEL*mMeters2Pixels;
  vote.pos.x = 0;
  vote.pos.y = -(PATCH_WIDTHS[STRAIGHT]*0.5 + PATCH_WIDTHS_PARKING_PARALLEL*0.5)*mMeters2Pixels;
  vote.weight = 1*mMinimumVote;   // Minimal vote. This is only used for lateral localization.
  //vote.weight = mMinimumVoteForDetectionParkingLot - 2*mMinimumVote;   // Minimal vote. This is only used for lateral localization.
  vote.size.height = width;
  vote.size.width = length;
  vote.angleOffset = 0;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.angleOffset = mAngleRegionSize;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.size.width = length*1.5;
  vote.angleOffset = 0;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  // Also for "cross" PARKING patches:
  width = 1.2*PATCH_LENGTHS_PARKING_CROSS*mMeters2Pixels;
  length = 0.02*mMeters2Pixels;
  vote.pos.y = 0;
  vote.pos.x = -(PATCH_WIDTHS[STRAIGHT]*0.5 + PATCH_LENGTHS_PARKING_CROSS*0.5)*mMeters2Pixels;
  vote.size.height = width;
  vote.size.width = length;
  //vote.weight = mMinimumVoteForDetectionParkingLot-2*mMinimumVote;
  vote.weight = 3*mMinimumVote;   // Minimal vote. This is only used for lateral localization.
  //vote.weight = mMinimumVoteForDetectionParkingLot - 2*mMinimumVote;   // Minimal vote. This is only used for lateral localization.
  vote.angleOffset = M_PI*0.5;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.angleOffset = M_PI*0.5+mAngleRegionSize;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.angleOffset = M_PI*0.5-mAngleRegionSize;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );

  // Same thing again for other side:
  width = 0.02*mMeters2Pixels;
  length = PATCH_LENGTHS_PARKING_PARALLEL*mMeters2Pixels;
  vote.pos.x = 0;
  vote.pos.y = (PATCH_WIDTHS[STRAIGHT]*0.5 + PATCH_WIDTHS_PARKING_PARALLEL*0.5)*mMeters2Pixels;
  vote.weight = 1*mMinimumVote;   // Minimal vote. This is only used for lateral localization.
  //vote.weight = mMinimumVoteForDetectionParkingLot - 2*mMinimumVote;   // Minimal vote. This is only used for lateral localization.
  vote.size.height = width;
  vote.size.width = length;
  vote.angleOffset = 0;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.angleOffset = mAngleRegionSize;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.angleOffset = -mAngleRegionSize;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.size.width = length*1.5;
  vote.angleOffset = 0;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  // Also for "cross" PARKING patches:
  width = 1.2*PATCH_LENGTHS_PARKING_CROSS*mMeters2Pixels;
  length = 0.02*mMeters2Pixels;
  vote.pos.y = 0;
  vote.pos.x = (PATCH_WIDTHS[STRAIGHT]*0.5 + PATCH_LENGTHS_PARKING_CROSS*0.5)*mMeters2Pixels;
  vote.size.height = width;
  vote.size.width = length;
  //vote.weight = mMinimumVoteForDetectionParkingLot-2*mMinimumVote;
  vote.weight = 3*mMinimumVote;   // Minimal vote. This is only used for lateral localization.
  //vote.weight = mMinimumVoteForDetectionParkingLot - 2*mMinimumVote;   // Minimal vote. This is only used for lateral localization.
  vote.angleOffset = M_PI*0.5;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.angleOffset = M_PI*0.5+mAngleRegionSize;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );
  vote.angleOffset = M_PI*0.5-mAngleRegionSize;
  mPatchVotesForParkingLot[STRAIGHT].push_back( vote );



  ////////////////////////////////////////////
  // Let CROSS_SECTION patches vote for STRAIGHT patches
  // in all four directions:
  width = 0.3*mMeters2Pixels;
  //length = 0.2*mMeters2Pixels;
  length = (PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*mMeters2Pixels*0.75*0.5;
  //vote.weight = mMinimumVoteForDetection-1*mMinimumVote;
  for( float angle = 0; angle < M_PI*2.0; angle += M_PI_2 )
  {
    vote.size.height = width;
    vote.size.width = length;
    vote.angleOffset = angle;
    vote.weight = mMinimumVoteForDetection;
    vote.pos.x = (PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5*mMeters2Pixels;
    vote.pos.y = 0;//(PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5;
    mPatchVotes[CROSS_SECTION].push_back( vote );
    vote.pos.x = -(PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5*mMeters2Pixels;
    vote.pos.y = 0;// (PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5;
    mPatchVotes[CROSS_SECTION].push_back( vote );
    
    vote.size.height = width;
    vote.size.width = length;
    vote.pos.x = (PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5*mMeters2Pixels + length;
    vote.pos.y = 0;//(PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5;
    mPatchVotes[CROSS_SECTION].push_back( vote );
    vote.pos.x = -(PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5*mMeters2Pixels - length;
    vote.pos.y = 0;// (PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5;
    mPatchVotes[CROSS_SECTION].push_back( vote );

    vote.size.height = 2.0*width;
    vote.size.width = length;
    vote.angleOffset = angle;
    vote.weight = mMinimumVoteForDetection - 2;
    vote.pos.x = (PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5*mMeters2Pixels;
    vote.pos.y = 0;//(PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5;
    mPatchVotes[CROSS_SECTION].push_back( vote );
    vote.pos.x = -(PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5*mMeters2Pixels;
    vote.pos.y = 0;// (PATCH_LENGTHS[CROSS_SECTION] + PATCH_LENGTHS[STRAIGHT])*0.5;
    mPatchVotes[CROSS_SECTION].push_back( vote );

  }
}

/*void StreetPatcher::initStreetPatchImages()
{
    ////////////////////////////////////////////
    // STRAIGHT STREET PATCH:
    cv::Mat straight = cv::Mat( STREET_FIELD_WIDTH*mMeters2Pixels,
                        STREET_FIELD_WIDTH*mMeters2Pixels, CV_8UC3, CV_RGB(0,0,0));
    float left = (1.0 - STREET_WIDTH)/2.0;
    float right = 1.0 - left;
    float top = (1.0 - PATCH_LENGTH)/2.0*mMeters2Pixels;
    cv::Point2f p1( left*mMeters2Pixels, top);
    cv::Point2f p2( right*mMeters2Pixels, top + PATCH_LENGTH*mMeters2Pixels );
    cv::rectangle( straight, p1, p2, CV_RGB( 0, 255, 0 ), 1 );

    mPatchImages.push_back( straight );

    //imwrite( "straightPatch.png", straight );
}*/

/*void StreetPatcher::rotateImg(cv::Mat& src, cv::Mat& dst, double angle )
{
    int len = std::max(src.cols, src.rows);
    cv::Point2f pt(len/2., len/2.);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle*180.0/M_PI, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len));
}*/

cv::Mat StreetPatcher::generateDebugImage()
{
  // Get image of the detected features:
  cv::Mat image = mHaarFilter.generateDebugImage();

  // Add the found patches as well:
  EnvironmentPtr env = Environment::getInstance();
  const PatchPtrList* previousPatches = env->getStreet();

  PatchPtrList::const_reverse_iterator it = previousPatches->rbegin();
  for( int i = 0; it != previousPatches->rend() && i < 3; it++, i++ )
  {
    cv::Point2f center = mCoordConverter->world2Pixel( mImgCarPose, (*it)->getPose() );
    cv::Scalar col( 0, 55 + 200*(*it)->getProbability(), 0 );

    cv::RotatedRect rRect = cv::RotatedRect( center,
                                             cv::Size2f( (*it)->getWidth()*mCoordConverter->getPixelsPerMeter(),
                                                         (*it)->getHeight()*mCoordConverter->getPixelsPerMeter() ),
                                             -((*it)->getPose().getYaw() - mImgCarPose.getYaw())*180.0/M_PI );

    cv::Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++)
      cv::line( image, vertices[i], vertices[(i+1)%4], col, 1 );

    cv::circle( image, center, 4, col, 2 );
  }

  it = mPatches.rbegin();
  for( ; it != mPatches.rend(); it++ )
  {
    // Draw the ORIGINAL (i.e. unmodified) patch:
    cv::Point2f center = mCoordConverter->world2Pixel( mImgCarPose, (*it)->mOriginalPose );
    cv::Scalar col( 0, 0, 55 + 200*(*it)->getProbability() );

    cv::RotatedRect rRect = cv::RotatedRect( center,
                                             cv::Size2f( (*it)->getWidth()*mCoordConverter->getPixelsPerMeter(),
                                                         (*it)->getHeight()*mCoordConverter->getPixelsPerMeter() ),
                                             -((*it)->getPose().getYaw() - mImgCarPose.getYaw())*180.0/M_PI );

    cv::Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++)
      cv::line( image, vertices[i], vertices[(i+1)%4], col, 1 );

    cv::circle( image, center, 4, col, 2 );
  }

  return image;
}

cv::Mat StreetPatcher::generateHoughSpaceDebugImage()
{
  cv::Mat colored( mDebugHoughSpace.size(), CV_16UC3 );
  cv::Mat colored8Bit( mDebugHoughSpace.size(), CV_8UC3 );
  //cv::cvtColor( mDebugHoughSpace, colored, CV_8UC3 );

  double min, max;
  cv::Point min_loc, max_loc;
  cv::minMaxLoc( mDebugHoughSpace, &min, &max, &min_loc, &max_loc);

  cv::cvtColor( 255*mDebugHoughSpace/max, colored, CV_GRAY2BGR );
  colored.convertTo(colored8Bit, CV_8UC3);

  int i = 0;
  for( PatchPtrList::iterator it = mPatches.begin(); it != mPatches.end(); it++, i++ )
  {
    float angleRegionPos = mCols*angle2AngleRegionID( (*it)->mOriginalPose.getYaw() );

    cv::Point2f pos;
    if( (*it)->getPatchType() == STRAIGHT )
    {
      pos = cv::Point2f(
              ((*it)->mOriginalPose.getX() - mCenterPose.getX())*mMeters2Pixels +
              mCenter + angleRegionPos,
              ((*it)->mOriginalPose.getY() - mCenterPose.getY())*mMeters2Pixels +
              mCenter + 3*mRows );
    }
    else if( (*it)->getPatchType() == CROSS_SECTION )
    {
      pos = cv::Point2f(
              ((*it)->mOriginalPose.getX() - mCenterPose.getX())*mMeters2Pixels +
              mCenter + angleRegionPos,
              ((*it)->mOriginalPose.getY() - mCenterPose.getY())*mMeters2Pixels +
              mCenter + 2*mRows );
    }
    else if( (*it)->getPatchType() == PARKING )
    {
      pos = cv::Point2f(
              ((*it)->mOriginalPose.getX() - mCenterPose.getX())*mMeters2Pixels +
              mCenter + angleRegionPos,
              ((*it)->mOriginalPose.getY() - mCenterPose.getY())*mMeters2Pixels +
              mCenter + mRows );
    }

    cv::Scalar col( 0, 0, 50 + 200*(1.0 - float(i)/float(mPatches.size()) ) );

    if( i % 3 == 0 )
      cv::circle( colored8Bit, pos, 4, col, 2 );
    if( i % 3 == 1 )
      cv::circle( colored8Bit, pos, 4, col, 2 );
    if( i % 3 == 2 )
      cv::circle( colored8Bit, pos, 4, col, 2 );
  }

  int width = colored8Bit.size().width - 1;
  cv::line( colored8Bit, cv::Point2i( 0, 3*mRows ), cv::Point2i( width, 3*mRows ),
      cv::Scalar( 128, 64, 64 ) );
  cv::line( colored8Bit, cv::Point2i( 0, 2*mRows ), cv::Point2i( width, 2*mRows ),
      cv::Scalar( 128, 64, 64 ) );
  cv::line( colored8Bit, cv::Point2i( 0, mRows ), cv::Point2i( width, mRows ),
      cv::Scalar( 64, 64, 64 ) );
  for( int i = 1; i < mNumUsedAngleRegions; i++ )
  {
    cv::line( colored8Bit, cv::Point2i( i*mCols, 0 ), cv::Point2i( i*mCols, mRows*4 ),
        cv::Scalar( 32, 32, 32 ) );
  }


  cv::Mat flipped;
  cv::flip( colored8Bit, flipped, 0 );

  return flipped;
}

// Convert from the angle region back to an angle:
float StreetPatcher::angleRegion2Angle( int regionID )
{
  float angleRegionSize = 2.0*M_PI/mNumAngleRegions;
  return angleRegionSize*regionID;
}

// Discretize the angle into mNumAngleRegion regions:
int StreetPatcher::angle2AngleRegionID( float angle )
{
  // Move by half a region (so that 0 is center of region)
  angle = angle + mAngleRegionSize*0.5;

  // Wrap into 0..2 Pi range:
  angle = angle - 2.0*M_PI * floor( angle / (2.0*M_PI) );

  int regionID = floor( angle / (2*M_PI) * mNumAngleRegions );

  // The above formula _can_ end up with regionID == mNumAngleRegions.
  // Catch this exception:
  if( regionID >= mNumAngleRegions )
    regionID -= 1;

  // Wrap into the first 4 regions:
  if( regionID >= mNumUsedAngleRegions )
  {
    regionID -= mNumUsedAngleRegions;
  }

  return regionID;
}
int StreetPatcher::getNeighbourAngleRegion( int regionID, int offset )
{
  regionID += offset;
  while( regionID >= mNumAngleRegions )
    regionID -= mNumAngleRegions;
  while( regionID < 0 )
    regionID += mNumAngleRegions;

  if( regionID >= mNumUsedAngleRegions )
  {
    regionID -= mNumUsedAngleRegions;
  }

  return regionID;
}

void StreetPatcher::clearPatches()
{
  mPatches.clear();
}

void StreetPatcher::insertTrafficSign( TrafficSignPtr trafficSign )
{
  //Vote vote = mPatchVotes[patch->getPatchType()][i];

  // Let the vote go into all angle regions (because determining the exact direction of a 
  // TrafficSign is difficult).

  // discretize the pose position and place it into the corresponding pixel:
  float y = (trafficSign->getPose().getY() - mCenterPose.getY() )*mMeters2Pixels + mCenter;
  float x = (trafficSign->getPose().getX() - mCenterPose.getX() )*mMeters2Pixels + mCenter;

  // Rotate vote according to patch angle, then add to Point (x,y):
  /*int xVote, yVote;
    xVote = x + cos(ang)*vote.pos.x - sin(ang)*vote.pos.y;
    yVote = y + sin(ang)*vote.pos.x + cos(ang)*vote.pos.y;*/

  int voteAmount = mMinimumVote*10;

  float radius = TRAFFIC_SIGN_DIST_TO_CROSS_SECTION*mMeters2Pixels;

  cv::ellipse( mHoughSpaceForTrafficSigns,
      cv::Point2f( x, y ),
      cv::Size( radius, radius ),
      trafficSign->getYaw()*180/M_PI,
      25, 65,
      cv::Scalar( voteAmount ), 
      0.15*mMeters2Pixels );

  cv::ellipse( mHoughSpaceForTrafficSigns,
      cv::Point2f( x, y ),
      cv::Size( radius, radius ),
      trafficSign->getYaw()*180/M_PI,
      35, 55,
      cv::Scalar( voteAmount + mMinimumVote ), 
      2 );
  cv::ellipse( mHoughSpaceForTrafficSigns,
      cv::Point2f( x, y ),
      cv::Size( radius, radius ),
      trafficSign->getYaw()*180/M_PI,
      40, 50,
      cv::Scalar( voteAmount + mMinimumVote ), 
      2 );

}

/*void StreetPatcher::setFramesPerMeter( float framesPerMeter )
{
        mCurrentFramesPerMeter = std::max( framesPerMeter, 0.01f );
        LOGGING_INFO( lanedetectionLogger, "Frames per meter set: "
                        << mCurrentFramesPerMeter << endl );
}*/

bool comparePatchesByDist( PatchPtr &a,  PatchPtr &b){
  return a->getTempDist() < b->getTempDist();
}

void StreetPatcher::setImage( cv::Mat image, const ExtendedPose2d &carPose )
{
  mHaarFilter.setImage( image );
  /* first argument says if crossroad lines are detected (no argument / default is no crossroad detection)
         * second argument toggles if parking lots are detected (no argument / default is no parking lot detection)
         * third argument toggles if crossroad corners are detected (no argument / default is no crossroad corner detection)
         * fourth argument sets the window size in which the features are detected (lower border of the window is same as image border and window is centered in x direction)
         *
         * detection features can be switched on and off during runtime
         */
  LOGGING_INFO( lanedetectionLogger, "Search Features Per line: SearchForCorosssections: "
                << mSearchForCrossSections<< " SearchForParkingLots: "<<mSearchForParkingLots << endl );
  mHaarFilter.calculateFeaturesPerLine(
        true,
        mSearchForParkingLots);


  if(getIsInitialized()) {
    LOGGING_INFO( lanedetectionLogger, "Haar Filter is initialized" << endl );
  }
  else {
    LOGGING_INFO( lanedetectionLogger, "Haar Filter is NOT initialized" << endl );
  }

  //mHaarFilter.calculateFeaturesGridBased();
  FeatureVector* features = mHaarFilter.getFeatures();

  // Remove old features ...
  clearFeatures();

  setCarPose( carPose );

  // Vote previous patches back into the hough space:
  /*for( PatchPtrList::reverse_iterator it = mPatches.rbegin(); it != mPatches.rend(); it++ )
  {
    addPreviousPatch( *it, false );
  }*/
  PatchPtrList* previousPatches = Environment::getInstance()->getStreet();
  if( previousPatches->size() > 0 )
  {
    PatchPtrList::const_reverse_iterator it;
    for( it = previousPatches->rbegin(); it != previousPatches->rend(); it++ )
    {
      addPreviousPatch( *it, true );
    }
  }

  // Make sure the Patch Votes don't go above the minimum vote for detection:
  for( int i = 0; i < mNumUsedAngleRegions; i++ )
  {
    mHoughSpace[i] = cv::min(mHoughSpace[i], mMinimumVoteForDetection - 3*mMinimumVote );
    mHoughSpaceForCrossSection[i] =
        cv::min(mHoughSpaceForCrossSection[i], mMinimumVoteForDetectionCrossSection - 10*mMinimumVote);
    //mHoughSpaceForParkingLot[i] =
    //cv::min(mHoughSpaceForParkingLot[i], mMinimumVoteForDetectionParkingLot - 3*mMinimumVote);
  }

  // Add the new features:
  addFeatures( features );

  // Add found traffic signs. Let them vote for CROSS_SECTIONS.
  TrafficSignPtrList trafficSigns = Environment::getInstance()->getTrafficSigns();
  for( TrafficSignPtrList::iterator it = trafficSigns.begin(); it != trafficSigns.end(); it++ )
  {
    // Only let Traffic signs vote which have been detected multiple times.
    if((*it)->getProbability() <= TRAFFIC_SIGN_MINIMUM_CONNECTION_PROBABLITY) 
    {
      continue;
    }
    int signType = (*it)->getSignType();
    if( signType == MARKER_ID_UNMARKEDINTERSECTION ||
        signType == MARKER_ID_STOPANDGIVEWAY ||
        signType == MARKER_ID_HAVEWAY ||
        signType == MARKER_ID_AHEADONLY ||
        signType == MARKER_ID_GIVEWAY ||
        signType == MARKER_ID_ROUNDABOUT )
    {
      insertTrafficSign( *it );
    }
  }

  // Now add the Traffic signs to the votes for CROSS_SECTIONs:
  for( int i = 0; i < mNumUsedAngleRegions; i++ )
  {
    // Add to each angle region:
    mHoughSpaceForCrossSection[i] += mHoughSpaceForTrafficSigns;
  }

  // Remove old patches ...
  clearPatches();
  // ... and search for new ones:
  findPatches( 3 );

  // Sort the patches by distance to the car. This way, close-by patches will be added
  // to the environment before those which are further away:
  for( PatchPtrList::iterator it = mPatches.begin(); it != mPatches.end(); it++ )
    (*it)->calcTempDist( carPose );
  mPatches.sort( comparePatchesByDist );

  //std::cout << "[StreetPatcher] Found " << mPatches.size() << " new patches." << std::endl;
  for( PatchPtrList::iterator it = mPatches.begin(); it != mPatches.end(); it++ )
  {
    /*std::cout << "Found patch:" << std::endl;
                std::cout << "\tAngle: " << (*it)->getPose().getYaw() << std::endl;
                std::cout << "\tPos: " << (*it)->getPose().getX() << " " << (*it)->getPose().getY() << std::endl;*/

    //oadrive::world::Patch envPatch( oadrive::world::STRAIGHT, mPatches[i].pose );
    //envPatch.setProbability( float(mPatches.size() - i)/float(mPatches.size()) );
    Environment::getInstance()->addPatch( *it );
  }


  // TEST ONLY!
  // TODO: remove!
  /*ParkingType parkingSpace = getParkingSpaceDirection( image );
        LOGGING_INFO( lanedetectionLogger, "[SM] \tStreetPatcher thinks we're in a " );
        if( parkingSpace == PARKING_TYPE_PARALLEL )
        {
                LOGGING_INFO( lanedetectionLogger, "PARKING_TYPE_PARALLEL parking lot." << endl );
        } else if( parkingSpace == PARKING_TYPE_CROSS ) {
                LOGGING_INFO( lanedetectionLogger, "PARKING_TYPE_CROSS parking lot." << endl );
        } else {
                LOGGING_INFO( lanedetectionLogger, "PARKING_TYPE_UNKNOWN parking lot." << endl );
        }*/
}


ParkingType StreetPatcher::getParkingSpaceDirection( cv::Mat& image)
{
  mHaarFilter.setImage( image );
  mHaarFilter.calculateFeaturesPerLine( true, true );
  FeatureVector* features = mHaarFilter.getFeatures();

  if( features->size() < mMinimumNumberFeaturesForParkingLotOrientation ){
    return PARKING_TYPE_UNKNOWN;
  }

  //    ExtendedPose2d poseSum;
  double yawMean = 0;


  FeatureVector::iterator it;
  for( it = features->begin(); it != features->end(); it++)
  {
    // Do not use parking lot features for calculation of orientation because their orientation is not as expected
    if((*it).type == PARKING_LOT) {
      continue;
    }

    //        Position2d unit1;
    float angle = (*it).pose.getYaw();

    double angleD = Environment::getInstance()->angleModPi(angle);

    if(angleD > M_PI/2){
      angleD = M_PI - angleD;
    }
    yawMean += angleD;

    /*unit1(0) = cos( angleD );
        unit1(1) = sin( angleD );
        
        poseSum.setX(poseSum.getX()+cos( angleD ));
        poseSum.setY(poseSum.getY()+sin( angleD ));

        
        LOGGING_INFO( lanedetectionLogger, "Yaw1: " << angleD/M_PI*180 << endl );
        LOGGING_INFO( lanedetectionLogger, "Yaw2: " << atan2(unit1(1),unit1(0))/M_PI*180 << endl << endl );*/


  }

  yawMean = yawMean / features->size();


  // double yawSum = atan2( poseSum.getY(), poseSum.getX() );
  // yawSum = mEnvironment->angleModPi(yawSum);

  if(yawMean < M_PI/4){
    LOGGING_INFO( lanedetectionLogger, "Parked Parallel" << endl );
    return PARKING_TYPE_PARALLEL; //parallel
  }else{
    LOGGING_INFO( lanedetectionLogger, "Parked Cross " << endl );
    return PARKING_TYPE_CROSS;//cross
  }

  return PARKING_TYPE_UNKNOWN;	// just in case...
}

void StreetPatcher::setBoostCrossSectionFeatures( bool boost )
{
  if( boost )
    LOGGING_INFO( lanedetectionLogger, "ENABLING BOOST OF HALT LINES." << endl );
  else
    LOGGING_INFO( lanedetectionLogger, "DISABLING BOOST OF HALT LINES." << endl );
  
  mBoostCrossSectionFeatures = boost;
}

}   // namespace
}   // namespace


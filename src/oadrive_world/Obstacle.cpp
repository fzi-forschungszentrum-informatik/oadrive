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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-16
 *
 */
//----------------------------------------------------------------------

#include "Obstacle.h"
#include "EnvObject.h"

using namespace oadrive::core;

namespace oadrive{
namespace world{


Obstacle::Obstacle(const ExtendedPose2d &pose, double width, double height, SensorType type)
  : EnvObject(pose,width,height)
  , mSensorType( type )
  , mKalmanFilter(2,2,0) //2 states (x,y) and 2 measurements (x,y)
  , mStatic( false )
{
  mKalmanFilter.transitionMatrix = (cv::Mat_<float>(2, 2) << 1, 0, 0, 1); //x = x and y = y
  cv::setIdentity(mKalmanFilter.measurementMatrix); // we also measure x and y so this matrix must be the identety
  cv::setIdentity(mKalmanFilter.processNoiseCov, cv::Scalar::all(1e-5));
  cv::setIdentity(mKalmanFilter.errorCovPost, cv::Scalar::all(1));
  mSawUS = 0;
  mSawDepth = 0;
  mSawTrafficSign = 0;
  mIsRelevantAcc = false;
  cv::Mat_<float> measurement(2,1);
  if(type == US){
    mSawUS = 1;
    measurement(0) = pose.getX();
    measurement(1) = pose.getY();
    mKalmanFilter.statePre =measurement;
    mTTL = TIME_TO_LIVE_US;
    setProbability(0.5);
  }
  else if(type == DEPTH)
  {

    mSawDepth = 1;
    measurement(0) = pose.getX();
    measurement(1) = pose.getY();
    mKalmanFilter.statePre =measurement;
    setProbability(0.9);
    mTTL = TIME_TO_LIVE_DEPTH;
  }
  else if(type == TRAFFICSIGN)
  {
    mSawTrafficSign = 1;
    measurement(0) = pose.getX();
    measurement(1) = pose.getY();
    mKalmanFilter.statePre =measurement;
    setProbability(1);
    mTTL = 100000;

  }
}

void Obstacle::update(const ExtendedPose2d &pos, SensorType type)
{
  cv::Mat_<float> measurement(2,1);
  measurement(0) = pos.getX();
  measurement(1) = pos.getY();
  cv::Mat estimated;
  if(type == US)
  {

    mSawUS++;
    if(mTTL<1)
    {
      mTTL++;
    }
    double probaility = getProbability();
    //increase probaility
    if((probaility+0.01)<1)
    {
      setProbability(probaility+0.01);
    }
    if(mSawTrafficSign == 0)
    {
      if(mSawDepth == 0 || getTimeSinceLastUpdate()>2)
      {
        //don't use Kalman with US sensor because the measurements are not uncorrelated
        setPose(pos);
      }
    }
  }
  else if(type == DEPTH)
  {
    //don't update Traffic Signs
    if(mSawTrafficSign == 0)
    {
      mSawDepth++;
      setPose(pos);
      mTTL = 20;
      //            cv::setIdentity(mKalmanFilter.measurementNoiseCov, cv::Scalar::all(0.05));
      //            estimated = mKalmanFilter.correct(measurement);
      //            setPose(oadrive::core::ExtendedPose2d(estimated.at<float>(0),estimated.at<float>(1),pos.getYaw())); //update pos of Env Object with pos of Kalman filter
      //            //TODO check this
      //            //update Kalmanfilter for the case, that 2 measurements will come without a prediction
      //            mKalmanFilter.statePre = mKalmanFilter.statePost;
      //            mKalmanFilter.errorCovPre = mKalmanFilter.errorCovPost;
    }

  }
}

int Obstacle::getTimeToLive(){
  return this->mTTL;
}
void Obstacle::setTimeToLive(int TTL){
  this->mTTL = TTL;
}




double Obstacle::getMaxSpeed(double distance){
  return distance/5.0;
}


void Obstacle::addEventRegion(EventRegionPtr evRegion){
  this->mEventRegions.push_back(evRegion);
  evRegion->setPose( toWorldCoords( evRegion->getLocalPose() ) );
}

int Obstacle::getSawUS()
{
  return mSawUS;
}

int Obstacle::getSawDepth()
{
  return mSawDepth;
}

int Obstacle::getSawTrafficSign()
{
  return mSawTrafficSign;
}
EventRegionPtrList* Obstacle::getEventRegions(){
  return &this->mEventRegions;
}

void Obstacle::setFree(SensorType type)
{
  double probaility = getProbability();
  if(type == DEPTH)
  {
    if(mSawTrafficSign == 0)
    {
      if((probaility-0.25)>0)
      {
        setProbability(probaility-0.25);
      }
    }
  }
}
bool Obstacle::getIsRelevantAccDebugOnly() const
{
  return mIsRelevantAcc;
}

void Obstacle::setIsRelevantAcc(bool value)
{
  mIsRelevantAcc = value;
}







void Obstacle::setPose( const ExtendedPose2d &pose )
{
  EnvObject::setPose( pose );

  // Update EventRegion position:
  if( mEventRegions.size() > 0 ){
    for( EventRegionPtrList::iterator it = mEventRegions.begin(); it != mEventRegions.end(); it++ )
    {
      (*it)->setPose( toWorldCoords( (*it)->getLocalPose() ) );
    }
  }
}




bool Obstacle::isDoNotDelete(){
  return mDoNotDelete;
}
void Obstacle::setDoNotDelete(bool del){
  mDoNotDelete = del;
}

}
}

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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-24
 *
 * Center-Piece for Oadrive, representtion of the current world.
 *
 */
//----------------------------------------------------------------------

#include "Environment.h"

#include <iostream>
#include <oadrive_core/Interpolator.h>
#include <oadrive_world/worldLogging.h>
#include <oadrive_trafficsign/aadc_roadSign_enums.h>
#include <oadrive_util/Broker.h>
#include <oadrive_util/Config.h>

#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <sstream>
#include <boost/lexical_cast.hpp>
#include <exception>

using namespace boost::posix_time;
using namespace oadrive::control;
using namespace oadrive::util;
using namespace oadrive::obstacle;

using icl_core::logging::endl;
using icl_core::logging::flush;

// Initialize as NULL-Pointer:
boost::shared_ptr<Environment> Environment::mInstance;

namespace oadrive{
namespace world{

EnvironmentPtr Environment::init( CoordinateConverter* coordConverter, DriverModule* driver,
    oadrive::util::Timer *timer )
{
  if( mInstance )
  {
    LOGGING_ERROR( worldLogger, "Cannot initialize another Environent!" << endl <<
        "\tEnvironment is a Singleton." << endl <<
        "\tCall init() only once, then use getInstance()." << endl );
  } else {
    mInstance = EnvironmentPtr( new Environment( coordConverter, driver, timer ) );
    LOGGING_INFO( worldLogger, "Constructed new Environment." << endl );
    if( getInstance()  )
    {
      LOGGING_INFO( worldLogger, "Constructed new Environment." << endl );
    } else {
      LOGGING_INFO( worldLogger, "Could not construct new Environment." << endl );
    }
  }
  return mInstance;
}

EnvironmentPtr Environment::reset()
{
  if( mInstance )
  {
    Timer* timer = mInstance->getTimer();
    DriverModule* driver = mInstance->getDriver();
    CoordinateConverter* coordConverter = mInstance->getCoordConverter();
    WorldEventListener* listener = mInstance->mEventListener;
    ExtendedPose2d lastCarPose = mInstance->getCarPose();
    // Resetting the java-way: (:P)
    mInstance = EnvironmentPtr( new Environment( coordConverter, driver, timer ) );
    mInstance->setEventListener(listener);
    mInstance->updateCarPose(lastCarPose);
    LOGGING_INFO( worldLogger, "Environment was reset." << endl );
  } else {
    LOGGING_ERROR( worldLogger, "Cannot reset Environent!" << endl <<
        "\tNo instance found. Call init() before." << endl );
  }
  return mInstance;
}

EnvironmentPtr Environment::getInstance()
{
  if( !mInstance )
  {
    LOGGING_ERROR( worldLogger, "Cannot get Environent Instance!" << endl <<
        "\tCall init() once, then use getInstance()." << endl );
  }
  return mInstance;
}

// Constructor. Is private!
Environment::Environment( CoordinateConverter* coordConverter, DriverModule* driver,
                          oadrive::util::Timer *timer )
  : mCar( new EnvObject( ExtendedPose2d( 0, 0, 0 ), CAR_WIDTH, CAR_LENGTH ) )
  , mCoordConverter( coordConverter )
  , mRememberObstacles( false )
  , mDriver( driver )
  , mPatchStitcher()
  , mMaxNumberOfUnsortedPatches( 20 )
  , mEventListener( NULL )
  , mTimer(timer)
  , mTrajDebugCounter(0)
  , mCurrentUSSensorLimits( LIMIT_FOR_DRIVING )
{
  LOGGING_INFO(worldLogger, "Environment created."<<endl);
  // Default street stitcher:
  mLane = LANE_RIGHT;
  mAccOn = true;
  mOverwriteAccMinSpeed = false;
  mNewAccMinSpeed = 0.15;
  mStorePosesBeforePatch = true;

  initUSSensorLimits();

  mDebugPointTime = microsec_clock::local_time();
  if(mTimer != NULL){
    LOGGING_INFO( worldLogger, "Registering Environment as timer event listener." << endl );
    mTimer->addListener( this );
    //mTimer->setTimer( 1000, TIMER_TYPE_REMOVE_OBSTACLES );
    mTimer->setTimer(1000, TIMER_TYPE_REMOVE_OLD_OBJECTS);
  }
}

Environment::~Environment()
{
  if( mTimer != NULL )
  {
    mTimer->removeListener( this );
  }
}

void Environment::setEventListener( WorldEventListener* listener )
{
  mEventListener = listener;
}


void Environment::updateCarPose(const ExtendedPose2d &carPose){

  getCar()->setPose( carPose );

  if( mPastCarPoses.size() == 0 )
    mPastCarPoses.push_back( carPose );

  checkEventRegions( carPose, mPastCarPoses.back() );

  updateHistory();
}

void Environment::checkEventRegions( const ExtendedPose2d &newPose, const ExtendedPose2d &previousPose )
{
  // TODO: Maybe handle case where we go through the whole event region within one frame
  // In this case, maybe we should trigger both EVENT_ENTERED_REGION und EVENT_EXITED_REGION.

  EventRegionPtrList::iterator it;
  for( it = mEventRegions.begin(); it != mEventRegions.end(); )
  {
    EventRegionPtr eventReg = *it;
    if( eventReg->isToDelete()){
      LOGGING_INFO(worldLogger, "Deleted one event Region " << eventReg->getId() <<endl);
      it = mEventRegions.erase(it);

    }else {
      if (!eventReg->getIsCarInside()) {
        if (eventReg->isPointInside(newPose)) {
          eventReg->setIsCarInside(true);
          mEventListener->eventRegionTriggered(eventReg, EVENT_ENTERED_REGION);
        } else {
          Position2d newPos = newPose.getPosition();
          Position2d prevPos = previousPose.getPosition();
          // checkIntersections returns the number of edges of the rectangular regions which
          // have been cut, so the following is true if the region has been entered AND
          // exited within the last frame.
          if (eventReg->checkIntersections(newPos, prevPos) > 1) {
            // Trigger both enter and then exit event:
            mEventListener->eventRegionTriggered(eventReg, EVENT_ENTERED_REGION);
            mEventListener->eventRegionTriggered(eventReg, EVENT_EXITED_REGION);
          }
        }
      } else {
        if (!eventReg->isPointInside(newPose)) {
          eventReg->setIsCarInside(false);
          mEventListener->eventRegionTriggered(eventReg, EVENT_EXITED_REGION);
        }
      }
      it++;
    }
  }
}



void Environment::addPatch(PatchPtr patch){

  // TODO: maybe don't call for every patch?
  bool sorted = mPatchStitcher.addStreetPatch( patch );
  /*if( !sorted )
  {
    mUnsortedPatches.push_back( patch );
    if( mUnsortedPatches.size() > mMaxNumberOfUnsortedPatches )
    {
      mUnsortedPatches.erase( mUnsortedPatches.begin() );
    }
  }*/
}

void Environment::addStreetPatch(PatchPtr streetPatch)
{
  bool wasEmpty = false;
  if( mStreet.size() == 0 )
    wasEmpty = true;

  if( streetPatch->getPatchType() != PARKING )
    mStreet.push_back( streetPatch );
  else
    mParkingLots.push_back( streetPatch );

  if( streetPatch->getPatchType() == CROSS_SECTION )
  {

    // Remove any STRAIGHT patches which overlap with the new CROSS_SECTION.
    for( size_t i = 1; i < std::min( (unsigned int)mStreet.size(), (unsigned int)10 ); )
    {
      PatchPtrList::reverse_iterator it = mStreet.rbegin();
      std::advance( it, i );
      PatchPtr patchToRemove = *it;
      if( patchToRemove->getPatchType() == STRAIGHT &&
          patchToRemove->checkOverlap( streetPatch ) )
      {
        // TODO: Also remove from history?
        // Check with David
        // Warning! After this line, it is no longer valid!
        mOpenPatches.remove( patchToRemove );
        removePatchFromList( patchToRemove, &mStreet );
        removePatchFromList( patchToRemove, &mPastCarPatches );
      } else {
        i++;
      }
    }
  }

  generateEventRegionsForPatch( streetPatch );

  connectPatchesAndTrafficSigns();

  // TODO: This must be checked by TrajectoryFactory, not here!
  if( mStreet.size() > 0 && wasEmpty )
    if( mEventListener )
      mEventListener->eventReadyToGeneratePatchTrajectory();
}

void Environment::generateEventRegionsForPatch( PatchPtr streetPatch )
{
  if( streetPatch->getPatchType() == CROSS_SECTION )
  {
    // Add all the event regions which we need for crossing:
    EventRegionPtr evRegHalt(new EventRegion( CROSS_SECTION_HALT, ExtendedPose2d(), streetPatch->getWidth() + CROSS_SECTION_HALT_OFFSET, streetPatch->getHeight()+ CROSS_SECTION_HALT_OFFSET));
    streetPatch->addEventRegion(evRegHalt);
    evRegHalt->setObjectOfInterest(streetPatch);
    addEventRegion( evRegHalt );

    EventRegionPtr evRegBlink(new EventRegion( CROSS_SECTION_BLINK, ExtendedPose2d(), streetPatch->getWidth() + CROSS_SECTION_BLINK_OFFSET, streetPatch->getHeight()+ CROSS_SECTION_BLINK_OFFSET));
    streetPatch->addEventRegion(evRegBlink);
    evRegBlink->setObjectOfInterest(streetPatch);
    addEventRegion( evRegBlink );

    EventRegionPtr evRegCenter(new EventRegion( CROSS_SECTION_CENTER, ExtendedPose2d(), streetPatch->getWidth() + CROSS_SECTION_CENTER_OFFSET, streetPatch->getHeight() + CROSS_SECTION_CENTER_OFFSET ));
    streetPatch->addEventRegion(evRegCenter);
    addEventRegion( evRegCenter );

    EventRegionPtr evRegObst1(
        new EventRegion( CROSS_SECTION_OBSTACLES,
          ExtendedPose2d(-PATCH_LENGTHS[CROSS_SECTION]*0.5-0.5,-STREET_SIDE_TO_MID_LANE+0.16,M_PI),
          streetPatch->getWidth()/2, streetPatch->getHeight()+1.0));
    streetPatch->addEventRegion(evRegObst1);
    addEventRegion( evRegObst1 );
    EventRegionPtr evRegObst2(
        new EventRegion( CROSS_SECTION_OBSTACLES,
          ExtendedPose2d(-STREET_SIDE_TO_MID_LANE+0.16,PATCH_LENGTHS[CROSS_SECTION]*0.5+0.5,M_PI/2),
          streetPatch->getWidth()/2, streetPatch->getHeight()+1.0));
    streetPatch->addEventRegion(evRegObst2);
    addEventRegion( evRegObst2 );
    EventRegionPtr evRegObst3(
        new EventRegion( CROSS_SECTION_OBSTACLES,
          ExtendedPose2d(STREET_SIDE_TO_MID_LANE-0.16,-PATCH_LENGTHS[CROSS_SECTION]*0.5-0.5,3*M_PI/2),
          streetPatch->getWidth()/2, streetPatch->getHeight()+1.0));
    streetPatch->addEventRegion(evRegObst3);
    addEventRegion( evRegObst3 );
    EventRegionPtr evRegObst4(
        new EventRegion( CROSS_SECTION_OBSTACLES,
          ExtendedPose2d(PATCH_LENGTHS[CROSS_SECTION]*0.5+0.5,STREET_SIDE_TO_MID_LANE-0.16,0),
          streetPatch->getWidth()/2, streetPatch->getHeight()+1.0));
    streetPatch->addEventRegion(evRegObst4);
    addEventRegion( evRegObst4 );
    EventRegionPtr evRegObstCenter(
        new EventRegion( CROSS_SECTION_OBSTACLES,
          ExtendedPose2d( 0,0,0),
          streetPatch->getWidth()-0.25, streetPatch->getHeight()-0.25));
    streetPatch->addEventRegion( evRegObstCenter );
    addEventRegion( evRegObstCenter );

  } else if( streetPatch->getPatchType() == PARKING ) {
    // Determine if it's a parallel or cross parking spot
    ExtendedPose2d carPose = getCar()->getPose();
    ExtendedPose2d patchPose = streetPatch->getPose();
    double angleDifference = angleModPi(patchPose.getYaw() - carPose.getYaw());

    ExtendedPose2d pose;
    EventRegionPtr evRegParallel;

    // if car pose and patch pose are perpendicular, it is a cross parking spot
    if(angleDifference > M_PI_4 && angleDifference < M_PI_2 + M_PI_4) {
      // compare orientation of car and patch, if they are too different, then the sign has to be flipped
      double angleDifferenceOrientation = angleMod2Pi(patchPose.getYaw() - (carPose.getYaw() + M_PI_2));
      // Check if patch is oriented similar as car or rotated by 180 degrees
      int sign = 1;
      if(angleDifferenceOrientation > M_PI_2 && angleDifferenceOrientation < M_PI + M_PI_2) {
        sign = -1;
      }

      // pose is in coordinate system of the patch
      double offsetX = 0.35;
      pose = ExtendedPose2d( sign * (PATCH_LENGTHS[PARKING] / 2 + PATCH_WIDTHS[PARKING] / 2), -offsetX / 2., M_PI_2 );
      evRegParallel = EventRegionPtr(
                        new EventRegion( PARKING_CROSS, pose, streetPatch->getWidth(),
                                         streetPatch->getWidth() + offsetX)
                        );
    }
    // otherwise it is a parallel parking spot
    else {
      // compare orientation of car and patch, if they are too different, then the sign has to be flipped
      double angleDifferenceOrientation = angleMod2Pi(patchPose.getYaw() - carPose.getYaw());
      // Check if patch is oriented similar as car or rotated by 180 degrees
      int sign = 1;
      if(angleDifferenceOrientation > M_PI_2 && angleDifferenceOrientation < M_PI + M_PI_2) {
        sign = -1;
      }

      // pose is in coordinate system of the patch
      double offsetX = 0.3;
      pose = ExtendedPose2d( sign*offsetX / 2., sign * PATCH_WIDTHS[PARKING], 0 );
      evRegParallel = EventRegionPtr(
                        new EventRegion( PARKING_PARALLEL, pose, streetPatch->getWidth(),
                                         streetPatch->getHeight() + offsetX)
                        );
    }

    // Generate event region next to the patch:
    streetPatch->addEventRegion(evRegParallel);
    evRegParallel->setObjectOfInterest(streetPatch);
    addEventRegion( evRegParallel );
  }

}

double Environment::angleModPi(double angle)
{
  return modulo(angle, M_PI);
}

double Environment::angleMod2Pi(double angle)
{
  return modulo(angle, 2 * M_PI);
}

double Environment::modulo(double value, double modulo) {
  // calculate value in range [0, modulo)
  int factor = value / modulo;
  double valueMod = value - factor * modulo;

  // check if value is positive, otherwise modulo has to be added once to put into range [0, modulo)
  if(valueMod < 0) {
    valueMod += modulo;
  }

  return valueMod;
}

void Environment::connectPatchesAndTrafficSigns()
{
  TrafficSignPtrList::iterator signIt;
  for( signIt = mTrafficSigns.begin(); signIt != mTrafficSigns.end(); signIt ++ )
  {
    // Do not connect traffic signs with low probability
    if((*signIt)->getProbability() <= TRAFFIC_SIGN_MINIMUM_CONNECTION_PROBABLITY) {
      continue;
    }

    int signType = (*signIt)->getSignType();
    PatchPtr closestPatch;
    float minDistance;
    int patchTypeToSearchFor;
    // Sign types possible at CROSS_SECTIONs:
    if( signType == MARKER_ID_UNMARKEDINTERSECTION ||
        signType == MARKER_ID_STOPANDGIVEWAY ||
        signType == MARKER_ID_HAVEWAY ||
        signType == MARKER_ID_AHEADONLY ||
        signType == MARKER_ID_GIVEWAY ||
        signType == MARKER_ID_ROUNDABOUT )
    {
      patchTypeToSearchFor = CROSS_SECTION;
      minDistance = TRAFFIC_SIGN_MAX_DIST_TO_CROSS_SECTION;
    }
    else	// Sign types possible "everywhere" (i.e. at STRAIGHT patches)
    {
      patchTypeToSearchFor = STRAIGHT;
      minDistance = TRAFFIC_SIGN_MAX_DIST_TO_STRAIGHT;
    }

    // Look for the closest patch which matches the criterium (closer than minDistance and
    // must be the correct patch type):
    PatchPtrList::iterator patchIt;
    for( patchIt = mStreet.begin(); patchIt != mStreet.end(); patchIt++ )
    {
      if( (*patchIt)->getPatchType() == patchTypeToSearchFor )
      {
        float distance = (*signIt)->calcDistTo( (*patchIt)->getPose() );
        if( distance < minDistance )
        {
          minDistance = distance;
          closestPatch = *patchIt;
        }
      }
    }

    if( closestPatch )
    {
      connect( *signIt, closestPatch );
    }
  }
}

bool Environment::connect( TrafficSignPtr sign, PatchPtr newPatch )
{
  PatchPtr oldPatch = sign->getConnectedPatch();
  if( !oldPatch || oldPatch != newPatch )
  {
    // First, disconnect any patches which were already connected:
    if( oldPatch )
    {
      oldPatch->removeTrafficSign( sign );
      // If an event region was attached to this patch, remove it:
      if( oldPatch->getEventRegion() )
      {
        EventRegionPtr eventRegionToBeDeleted = oldPatch->getEventRegion();
        oldPatch->removeEventRegion( eventRegionToBeDeleted );
        removeEventRegion( eventRegionToBeDeleted );
      }
    }
    // Let the sign know it is now "attached" to this patch:
    sign->setConnectedPatch( newPatch );

    // Let the patch also track the sign:
    newPatch->addTrafficSign( sign );

    // If this is a parking area sign, add an event region:
    if( sign->getSignType() == MARKER_ID_PARKINGAREA )
    {
      double angleTrafficSign2PatchMod2Pi = angleMod2Pi(sign->getPose().getYaw() - newPatch->getPose().getYaw());

      bool sameOrientation = true;
      if(angleTrafficSign2PatchMod2Pi < M_PI_2 || angleTrafficSign2PatchMod2Pi > M_PI + M_PI_2) {
        sameOrientation = false;
      }

      ExtendedPose2d pose( -1, -newPatch->getWidth()*0.25, 0 );
      if(!sameOrientation) {
        pose = ExtendedPose2d( 1, newPatch->getWidth()*0.25, 0 );
      }
      //EventRegionPtr evRegion( new EventRegion( PARKING_SIGN, pose,
      //newPatch->getWidth()*0.7, newPatch->getHeight()+2 ) );
      //newPatch->addEventRegion( evRegion );
      EventRegionPtr evRegion( new EventRegion( PARKING_SIGN, pose,
                                                newPatch->getWidth()*0.7, newPatch->getHeight()+2.5 ) );
      //sign->addEventRegion( evRegion );
      newPatch->addEventRegion( evRegion );
      addEventRegion( evRegion );
    }

    EventRegionPtrList eventRegions = sign->getEventRegions();
    for(EventRegionPtrList::const_iterator evRegionIt = eventRegions.begin(), evRegionEnd = eventRegions.end(); evRegionIt != evRegionEnd; ++evRegionIt) {
      if((*evRegionIt)->getEventRegionType() == UNCONNECTED_TRAFFIC_SIGN) {
        removeEventRegion( *evRegionIt );
        sign->clearEventRegions();
      }
    }

    return true;
  } else {
    return false;
  }
}

void Environment::addEventRegionToTrafficSign(TrafficSignPtr sign) {
  // If we have a parking sign and no patch is attached to this sign so far, attach an event region to this sign
  if( sign->getSignType() == MARKER_ID_PARKINGAREA && !sign->getConnectedPatch() ) {
    EventRegionPtrList eventRegions = sign->getEventRegions();
    for(EventRegionPtrList::iterator evRegionIt = eventRegions.begin(); evRegionIt != eventRegions.end(); evRegionIt++) {
      if((*evRegionIt)->getEventRegionType() == PARKING_SIGN) {
        removeEventRegion( *evRegionIt );
        sign->removeEventRegion( *evRegionIt );
      }
    }

    ExtendedPose2d pose( 1, 0.75, 0 );

    EventRegionPtr evRegion( new EventRegion( PARKING_SIGN, pose,
                                              1.75, 3.5 ) );
    sign->addEventRegion( evRegion );
    addEventRegion( evRegion );
  }
}

bool compareEnvObjects(EnvObjectPtr &a, EnvObjectPtr &b){
  return a->getTempDist() < b->getTempDist();
}
bool compareObstacles(ObstaclePtr &a, ObstaclePtr &b){
  return a->getTempDist() < b->getTempDist();
}
bool compareTrafficSigns(TrafficSignPtr &a, TrafficSignPtr &b){
  return a->getTempDist() < b->getTempDist();
}

EnvObjectPtr Environment::findNearestInList( const ExtendedPose2d &pose,
                                             EnvObjectPtrList* envObjects ) {

  // If empty, return NULL pointer:
  if( envObjects->size() == 0 )
    return EnvObjectPtr();

  for( EnvObjectPtrList::iterator it = envObjects->begin(); it != envObjects->end(); it++ )
  {
    (*it)->calcTempDist( pose );
  }
  envObjects->sort(compareEnvObjects);

  return envObjects->front();
}

TrafficSignPtr Environment::getNearestTrafficSign( const ExtendedPose2d &pose, int signType )
{
  // If empty, return NULL pointer:
  if( mTrafficSigns.size() == 0 )
    return TrafficSignPtr();

  TrafficSignPtr closest;
  double minDistance = std::numeric_limits<double>::max();

  TrafficSignPtrList::iterator it;
  for( it = mTrafficSigns.begin(); it != mTrafficSigns.end(); it++ )
  {
    if( signType == (*it)->getSignType() )
    {
      double distance = (*it)->calcDistTo( pose );
      if( distance < minDistance )
      {
        minDistance = distance;
        closest = (*it);
      }
    }
  }

  return closest;
}

void Environment::addDebugPoint( const ExtendedPose2d &pose, cv::Scalar col,
                                 double secondsUntilDeletion, bool drawDirection )
{
  DebugPoint p;
  p.col = col;
  p.drawDirection = drawDirection;
  p.pose = pose;
  p.time = microseconds( secondsUntilDeletion*1e6 );
  mDebugPoints.push_back( p );
}

void Environment::addDebugPoint( const Position2d &pos, cv::Scalar col,
                                 double secondsUntilDeletion )
{
  DebugPoint p;
  p.col = col;
  p.drawDirection = false;
  p.pose = ExtendedPose2d( pos[0], pos[1], 0 );
  p.time = microseconds( secondsUntilDeletion*1e6 );
  mDebugPoints.push_back( p );
}


/*void Environment::collectDebugPoints()
{
  addDebugPoint(mDriver->getLateralController()->getProjectedPose(),cv::Scalar(64,85,255),5,true);
}*/

Trajectory2d Environment::getTrajectory()
{
  return mTrajectory;
}

void  Environment::updateHistory(){

  //get current Patches the car is on
  PatchPtrList newCarPatches;

  for ( PatchPtrList::iterator it = mStreet.begin(); it != mStreet.end(); it++ )
  {
    PatchPtr patch = *it;
    if(patch->isPointInside(getCar()->getPose())){
      newCarPatches.push_back(patch);
    }
  }

  //check if car is on a new patch
  bool isNew;
  for(PatchPtrList::iterator it = newCarPatches.begin();it != newCarPatches.end(); it++){
    PatchPtr patchNew = *it;
    isNew = 1;
    for(PatchPtrList::iterator it2 = mCurrentCarPatches.begin();it2 != mCurrentCarPatches.end(); it2++){
      PatchPtr patchOld = *it2;
      if(patchNew->getId() == patchOld->getId()){
        isNew = 0;
      }
    }
    //add new patch to the past patches the car was on
    if(isNew){

      mPastCarPatches.push_back(patchNew);

      int size = mPastCarPoses.size() - 1;
      if(mStorePosesBeforePatch)
        mCarPoseIndexBeforePatch[patchNew->getId()] = std::max(0,size);

      setCarHasBackedUp(false);

    }

  }

  //add new car pos to history if car moved more than the CAR_DIFF_POS_THRES threshold
  bool poseChange = 0;
  ExtendedPose2d lastPos = mPastCarPoses.back();
  if(lastPos.distance(getCar()->getPose()) > CAR_DIFF_POS_THRES){
    mPastCarPoses.push_back(getCar()->getPose());
    poseChange = 1;
  }

  //update the patches the car is currently on
  mCurrentCarPatches = newCarPatches;

  //update car poses on current Patches
  if(poseChange && mStorePosesBeforePatch){
    for(PatchPtrList::iterator it = mCurrentCarPatches.begin();it != mCurrentCarPatches.end(); it++){
      PatchPtr patch = *it;
      mCarPosesOnPatch[patch->getId()].push_back(getCar()->getPose());

    }

  }

}
//

void Environment::removePatchFromList(PatchPtr patch, PatchPtrList* list){

  bool found = (std::find(list->begin(), list->end(), patch) != list->end());

  if(found){
    
    PatchPtrList children = patch->getChildren();

    PatchPtrList::iterator child;
    for( child = children.begin(); child != children.end(); child++ )
    {
      (*child)->addChildren( children );
      (*child)->removeChild( patch );
    }

    EventRegionPtrList evRegions = patch->getEventRegions();
    for( EventRegionPtrList::iterator it = evRegions.begin(); it != evRegions.end(); it++ )
    {
      removeEventRegion( *it );
    }

    list->remove(patch);
  }

}


void Environment::removeOldStreetPatches(int seconds){

  mOpenPatches.clear();
  int time = std::time(0);
  for( unsigned int i = 0; i < getStreet()->size(); )
  {
    PatchPtrList::iterator it = getStreet()->begin();
    std::advance( it, i );
    PatchPtr patch = *it;
    if(time - patch->getTime() > seconds &&
       patch->calcDistTo( getCar() ) > OLD_PATCHES_MIN_DIST )
    {
      mOpenPatches.remove( patch );
      removePatchFromList( patch, &mStreet );
      removePatchFromList( patch, &mPastCarPatches );
    } else {
      if( patch->hasOpenSides() )
      {
        mOpenPatches.push_back( patch );
      }
      i++;
    }
  }

}


void Environment::removeOldObjects(){

  removeOldStreetPatches(OLD_PATCHES_LIFE_TIME);
  removeOldTrafficSigns();

  if(mTimer != NULL)
    mTimer->setTimer(1000, TIMER_TYPE_REMOVE_OLD_OBJECTS);

}


void Environment::removeOldTrafficSigns(){

  int time = std::time(0);
  for( TrafficSignPtrList::iterator it = mTrafficSigns.begin(); it != mTrafficSigns.end();  ){

    TrafficSignPtr sign = *it;
    if(time - sign->getTime() > OLD_TRAFFICSIGN_LIFE_TIME &&
      sign->calcDistTo( getCar() ) > OLD_TRAFFICSIGN_MIN_DIST){

      EventRegionPtrList eventRegions = sign->getEventRegions();
      for(EventRegionPtrList::const_iterator evRegionIt = eventRegions.begin(), evRegionEnd = eventRegions.end(); evRegionIt != evRegionEnd; ++evRegionIt) {
        if((*evRegionIt)->getEventRegionType() == UNCONNECTED_TRAFFIC_SIGN) {
          removeEventRegion( *evRegionIt );
          sign->clearEventRegions();
        }
      }

      it = mTrafficSigns.erase(it);
    }
    else{
      it++;
    }
  }
}



void Environment::clearAllPatches(){

  for ( PatchPtrList::iterator it = mStreet.begin(); it != mStreet.end(); it++ )
  {
    if( (*it)->getEventRegions().size() > 0 ){
      for( EventRegionPtrList::iterator it2 = (*it)->getEventRegions().begin(); it2 != (*it)->getEventRegions().end(); it2++ )
      {
        mEventRegions.remove(*it2);
      }
    }
  }
  for ( PatchPtrList::iterator it = mParkingLots.begin(); it != mParkingLots.end(); it++ )
  {
    if( (*it)->getEventRegions().size() > 0 ){
      for( EventRegionPtrList::iterator it2 = (*it)->getEventRegions().begin(); it2 != (*it)->getEventRegions().end(); it2++ )
      {
        mEventRegions.remove(*it2);
      }
    }
  }
  //mUnsortedPatches.clear();
  mStreet.clear();
  mOpenPatches.clear();
  mCurrentCarPatches.clear();
  mPastCarPatches.clear();
  mParkingLots.clear();

}

void Environment::clearObstacles( SensorType type ){


  LOGGING_INFO(worldLogger, " clear Obstacles starting ... " <<endl);
  ObstaclePtrList::iterator it;
  for( it = mObstacles.begin(); it != mObstacles.end(); )
  {
    // Only delete if
    //  a) correct sensor type and
    //  b) we're not currently remembering obstacles. If we are, don't
    //    remove static obstacles.
    if( (*it)->getSensorType() == type &&
        ( mRememberObstacles == false || (*it)->isStatic() == false ) )
    {
      // Delete the obstacle's event regions:
      if( (*it)->getEventRegions()->size() > 0 )
      {
        EventRegionPtrList::iterator itEventRegion;
        for( itEventRegion = (*it)->getEventRegions()->begin();
            itEventRegion != (*it)->getEventRegions()->end();
            itEventRegion++ )
        {
          removeEventRegion(*itEventRegion);
        }
      }
      it = mObstacles.erase( it );
    } else {
      it ++;
    }
  }
  LOGGING_INFO(worldLogger, " Cleared Obstacles. " <<endl);
}

void Environment::setTrajectory( const MultiTrajectory& traj )
{
  mMultiTrajectory = traj;

  updateMultiTrajSpeed();

  bool canDriveNewTrajectory = false;
  if( mDriver )
    canDriveNewTrajectory = mDriver->setTrajectory( mMultiTrajectory );

  /*if(mTrajectory.size()<2)
    {
    LOGGING_INFO(worldLogger, "Trajectory has less than 2 Points"<<endl);
    }
    else
    {
    mTrajectorySimple = mTrajectory.simplify(0.1);
    mTrajectorySimple.calculateOrientations();
    }*/

  if( mEventListener )
  {
    if( canDriveNewTrajectory )
    {
      if(isEndOfTrajReached())
      {
        setEndOfTrajReached(false);
        mEventListener->eventReadyToGeneratePatchTrajectory();
      }
    } else {
      mEventListener->eventTrajectoryEmpty();
    }
  }

  // Write trajectories to disk for debug reasons
  /*std::stringstream ss;
    ss<<"/tmp/recordedData/Trajectories/"<< std::setfill('0') << std::setw(5)<<mTrajDebugCounter<<".txt";
    std::ofstream file;
    file.open(ss.str().c_str());
    file << traj;
  file.close();
  mTrajDebugCounter++;*/
}


void Environment::setTrajectory( const Trajectory2d& traj )
{
  MultiTrajectory multiTraj;
  multiTraj.trajectories.push_back( traj );
  setTrajectory( multiTraj );
}


bool Environment::isCarAtTrajectoryEnd( const Trajectory2d& traj ){

  double REACHED_ZONE_DISTANCE = 0.35;

  if(traj.size() >= 2) {
    ExtendedPose2d projection;
    double distance;
    std::size_t nearest_pose_index;

    calculateProjection(traj, getCarPose(), projection, distance, nearest_pose_index);

    // check if at end of trajectory
    const bool position_good_enough =
            distance < 0.5 && (projection.getYaw() - getCarPose().getYaw()) < M_PI / 3;

    //do some more angle magic
    ExtendedPose2d lastPose = traj.back();
    ExtendedPose2d sndLastPose = traj[traj.size() - 2];
    ExtendedPose2d vecPose = getCarPose();


    double x1 = sndLastPose.getX() - lastPose.getX();
    double x2 = vecPose.getX() - lastPose.getX();

    double y1 = sndLastPose.getY() - lastPose.getY();
    double y2 = vecPose.getY() - lastPose.getY();

    double denom1 = std::max(sqrt(x1 * x1 + y1 * y1), 0.000001);
    double denom2 = std::max(sqrt(x2 * x2 + y2 * y2), 0.000001);
    x1 = x1 / denom1;
    y1 = y1 / denom1;
    x2 = x2 / denom2;
    y2 = y2 / denom2;
    double angle = acos(x1 * x2 + y1 * y2);

    bool behind = true;
    if (angle < M_PI / 2) {
      behind = false;
    }


    if (position_good_enough &&
        (projection.getPosition() - traj.back().getPosition()).norm() < REACHED_ZONE_DISTANCE && behind) {
      return true;
    }

    return false;

  }
  else{
    return true;
  }

}

bool Environment::isAtMultiTrajEnd( const MultiTrajectory& traj ){

  if(traj.trajectories.size() > 0){

    return isCarAtTrajectoryEnd(traj.trajectories.back());

  }

  return true;

}


void Environment::addEventRegion(EventRegionPtr eventReg){
  mEventRegions.push_back(eventReg);
}
void Environment::removeEventRegion(EventRegionPtr eventReg){
  eventReg->setToDelete(true);
}

void Environment::addObstacle(ObstaclePtr &obstacle)
{
  bool isNewObstacle = false;
  if(obstacle)
  {

    // Push the obstacle away from the car:
    Position2d diffVec = obstacle->getPose().getPosition() - getCar()->getPose().getPosition();
    diffVec.normalize();
    Position2d newPos = obstacle->getPose().getPosition() + diffVec*0.05;
    ExtendedPose2d newPose = obstacle->getPose();
    newPose.setPosition( newPos );
    obstacle->setPose( newPose );

    if( mRememberObstacles )
    {
      bool merged = false;
      ObstaclePtrList::iterator it;
      for( it = mObstacles.begin(); it != mObstacles.end(); it ++ )
      {
        if( (*it)->calcDistTo( obstacle ) < OBSTACLE_MERGING_DIST )
        {
          (*it)->setProbability( (*it)->getProbability() + obstacle->getProbability() );
          (*it)->setPose( obstacle->getPose() );
          merged = true;
          break;
        }
      }
      isNewObstacle = (merged == false);
    } else {
      isNewObstacle = true;
    }
  } else {
    return;
  }

  // If the obstacle was not merged into another one, add it to the environment.
  // In this case, also generate its event regions.
  if( isNewObstacle )
  {
    mObstacles.push_back(obstacle);

    EventRegionPtr evRegObst(new EventRegion( OBSTACLE_REGION , ExtendedPose2d(), obstacle->getWidth() + OBSTACLE_REGION_BORDER, obstacle->getHeight()+ OBSTACLE_REGION_BORDER));
    EventRegionPtr evRegObst2(new EventRegion( OBSTACLE_REGION_SMALL , ExtendedPose2d(), obstacle->getWidth() + OBSTACLE_REGION_BORDER_SMALL, obstacle->getHeight()+ OBSTACLE_REGION_BORDER_SMALL));
    obstacle->addEventRegion(evRegObst);
    obstacle->addEventRegion(evRegObst2);
    evRegObst->setObjectOfInterest(obstacle);
    evRegObst2->setObjectOfInterest(obstacle);
    addEventRegion( evRegObst );
    addEventRegion( evRegObst2 );

    // Check if the obstacle falls into a region where it should be
    // considered static (i.e. should not be deleted). This is usually
    // next to the street, i.e. on a parking lot.
    if( mRememberObstacles )
    {
      if( couldBeOnParkingLot( obstacle )  && obstacle->calcDistTo(getCar()) < 1.5 )
      {
        obstacle->setStatic( true );
      }
    }
  }

  /*if(isMultiRelevantObstacle(obstacle))
  {
    updateMultiTrajSpeed();

    if( mDriver )
      mDriver->setTrajectory( mMultiTrajectory ); 

  }*/
}

int sign( float x )
{
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

bool Environment::couldBeOnParkingLot( ObstaclePtr obstacle )
{
  ExtendedPose2d carPose = getCarPose();

  float c = cos( carPose.getYaw() );
  float s = sin( carPose.getYaw() );

  // Set up a line from the rear to the front, passing by the car on the right side:
  Position2d Alocal, Blocal, A, B;
  // right rear:
  Alocal(0) = -(CAR_LENGTH - CAR_ORIGIN_TO_FRONT);
  Alocal(1) = -CAR_WIDTH*0.5;

  // rotate:
  A(0) = carPose.getX() + Alocal(0)*c - Alocal(1)*s;
  A(1) = carPose.getY() + Alocal(0)*s + Alocal(1)*c;
  // right front:
  Blocal(0) = CAR_ORIGIN_TO_FRONT;
  Blocal(1) = -CAR_WIDTH*0.5;
  // rotate:
  B(0) = carPose.getX() + Blocal(0)*c - Blocal(1)*s;
  B(1) = carPose.getY() + Blocal(0)*s + Blocal(1)*c;

  addDebugPoint( A, cv::Scalar( 255,255,255 ), 1 );
  addDebugPoint( B, cv::Scalar( 200,200,255 ), 1 );

  // This magic line returns 0, -1 or +1:
  //  0: obstacle is on the line AB
  //  -1: obstacle is right of AB
  //  +1: obstacle is left of AB
  // Taken from Stack-Overflow:
  //  http://stackoverflow.com/questions/1560492/
  //      how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
  int position = sign((B(0) - A(0)) * (obstacle->getY() - A(1)) - (B(1) - A(1)) * (obstacle->getX() - A(0)));
  return (position == -1);
}

//TODO obstacle mutex
bool Environment::isObstacleFree(EnvObjectPtr object, bool checkNonStaticObstacles )
{
  if(!object)
  {
    LOGGING_WARNING(worldLogger, "Checking non-existant EnvObject for obstacles (2)."<<endl);
    return true;
  }

  bool free = true;

  for(ObstaclePtrList::iterator it = mObstacles.begin(); it != mObstacles.end(); it++){
    ObstaclePtr obstacle = *it;
    if( checkNonStaticObstacles || obstacle->getProbability() > EXISTENCE_THRES){
      if(obstacle->checkOverlap(object)){
        free = false;
        break;
      }
    }
  }

  return free;
}



bool Environment::isObstacleFree(EnvObjectPtr other, drivingDirection dd, bool checkNonStaticObstacles )
{
  if(!other)
  {
    LOGGING_WARNING(worldLogger, "Checking non-existant EnvObject for obstacles (1)."<<endl);
    return false;
  }

  PatchPtr patch;
  try {
    patch = boost::dynamic_pointer_cast<Patch>(other);
  } catch (std::exception &e) {
    LOGGING_ERROR(worldLogger, "EnvObject is not a Patch!"<<endl);
    return true;
  }
  EventRegionPtr obstReg = patch->getObstacleRegion(getCar()->getPose(), dd);
  return isObstacleFree( obstReg, checkNonStaticObstacles );
}


//Returns the index of the point which is nearest to the car (and in front of the car)
int Environment::getNextTrajIndex(){

  return getNextTrajIndex(getCar()->getPose());

}

//Returns the index of the point which is nearest to the car (and in front of the car)
int Environment::getNextTrajIndex(const ExtendedPose2d other){


  if(mTrajectory.size() <=0){
    return 0;
  }

  EnvObjectPtr car(new EnvObject(other));

  unsigned int counter = 0;
  for (Trajectory2d::iterator it=mTrajectory.begin(); it != mTrajectory.end(); it++)
  {
    ExtendedPose2d curPose = *it;
    double angle = car->calcAngleOffset(curPose);
    // angle to next Traj point <90° --> traj point is in front of car
    if(angle < M_PI/2.0){
      break;
    }
    counter++;
  }

  if(counter >= mTrajectory.size()) {
    counter = mTrajectory.size() -1;
  }

  return counter;
}

    //Returns the index of the point which is nearest to the car. (Nearest before the car)
int Environment::getNextTrajIndexSimple(){


  int counter = 0;
  for (Trajectory2d::iterator it=mTrajectorySimple.begin(); it != mTrajectorySimple.end(); it++)
  {
    ExtendedPose2d curPose = *it;
    double angle = getCar()->calcAngleOffset(curPose);
    // angle to next Traj point <90° --> traj point is in front of car
    if(angle < M_PI/2){
      break;
    }
    counter++;
  }
  return counter;


}

void Environment::addTrafficSign(TrafficSignPtr sign){

  // Only allow traffic signs which are close to the car. Anything else was too inprecise.
  if( mCar->calcDistTo( sign ) > TRAFFIC_SIGN_DETECTION_RADIUS )
  {
    return;
  }

  bool merged = false;
  bool connectNewPatches = false;
  TrafficSignPtr nearest = getNearestTrafficSign( sign->getPose(), sign->getSignType() );

  if( nearest )
  {
    if( nearest->calcDistTo( sign->getPose() ) < TRAFFIC_SIGN_MERGING_DIST )
    {
      nearest->setX( sign->getX() );
      nearest->setY( sign->getY() );
      double probability = nearest->getProbability();
      nearest->setProbability(probability + TRAFFIC_SIGN_PROBABILITY_ADD);
      if(probability + TRAFFIC_SIGN_PROBABILITY_ADD > TRAFFIC_SIGN_MINIMUM_CONNECTION_PROBABLITY) {
        connectNewPatches = true;
      }

      // Use likelyhood to find average of angle:
      // (Likelyhood is similar to probability, but not limited to 1).
      double likelyhood = nearest->getLikelyhood();
      Position2d vec1, vec2, vecAverage;
      vec1(0) = cos( nearest->getYaw() );
      vec1(1) = sin( nearest->getYaw() );
      vec2(0) = cos( sign->getYaw() );
      vec2(1) = sin( sign->getYaw() );
      vecAverage = likelyhood*vec1 + TRAFFIC_SIGN_PROBABILITY_ADD*vec2;
      double newAngle = atan2( vecAverage(1), vecAverage(0) );
      nearest->setLikelyhood( nearest->getLikelyhood() + TRAFFIC_SIGN_PROBABILITY_ADD);
      nearest->setYaw( newAngle );

      //LOGGING_INFO( worldLogger, "traffic sign Obstacle Updated: " << sign->getSignType() << endl );
      merged = true;
      LOGGING_INFO( worldLogger, "Merged traffic sign: " << sign->getSignType() << endl );
    }
  }

  if( !merged )
  {
    sign->setProbability(TRAFFIC_SIGN_PROBABILITY_ADD);
    sign->setLikelyhood(TRAFFIC_SIGN_PROBABILITY_ADD);
    mTrafficSigns.push_back( sign );

    //ObstaclePtr obst(new Obstacle(sign->getPose(), 0.05, 0.05, TRAFFICSIGN));
    //obst->setProbability(0.5);
    //mObstacles.push_back(obst);
    //addObstacle( obst );
    //sign->setObstacle(obst);

    LOGGING_INFO( worldLogger, "Added new traffic sign: " << sign->getSignType() << endl );
  }

  if(connectNewPatches) {
    /* add an event region to the traffic sign, if it's a parking sign
     * this was introduced since it can happen, that the traffic sign is too far away from
     * the starting point of the parking areas that it could happen that the first parking
     * areas are not detected since no patch close to the traffic sign is found at that time
     */
    addEventRegionToTrafficSign( nearest );

    connectPatchesAndTrafficSigns();

    // We found a pedestrians crossing traffic sign.
    if( nearest->getSignType() == MARKER_ID_PEDESTRIANCROSSING )
    {
      if( ! nearest->getEventRegion() )
      {
        // Generate a event region, halt when we enter it.
        EventRegionPtr evRegHalt( new EventRegion( PED_CROSSING_HALT,
              ExtendedPose2d( 0.5 * (0.35 + 1 + CAR_ORIGIN_TO_FRONT), 0.5 + 0.135, 0 ), // approx pos of sign from patch
              2.0, // width
              0.35 + 1 + CAR_ORIGIN_TO_FRONT )); // height
        evRegHalt->setObjectOfInterest( nearest );
        addEventRegion( evRegHalt );
        nearest->addEventRegion( evRegHalt );
      }
    }

    // If the sign could not be connected and it is a sign for a crossing, then (more
    // specifically a 'haveway' sign), add an event region. This means we couldn't find a
    // CROSS_SECTION, but we've found a corresponding traffic sign.
    /*if( ! nearest->getConnectedPatch() )
    {
      if( ! nearest->getEventRegion() )
      {
        if( nearest->getSignType() == MARKER_ID_UNMARKEDINTERSECTION ||
            nearest->getSignType() == MARKER_ID_STOPANDGIVEWAY ||
            nearest->getSignType() == MARKER_ID_HAVEWAY ||
            nearest->getSignType() == MARKER_ID_AHEADONLY ||
            nearest->getSignType() == MARKER_ID_GIVEWAY ||
            nearest->getSignType() == MARKER_ID_ROUNDABOUT ||
            //nearest->getSignType() == MARKER_ID_ONEWAYSTREET ||
            nearest->getSignType() == MARKER_ID_NOOVERTAKING )
        {
          ExtendedPose2d position(
              UNCONNECTED_TRAFFIC_SIGN_OFFSET,
              UNCONNECTED_TRAFFIC_SIGN_OFFSET,
              0 );
          EventRegionPtr evRegion( new EventRegion( UNCONNECTED_TRAFFIC_SIGN, position,
                UNCONNECTED_TRAFFIC_SIGN_REGION_SIZE,
                UNCONNECTED_TRAFFIC_SIGN_REGION_SIZE ) );
          addEventRegion( evRegion );
          nearest->addEventRegion( evRegion );
          evRegion->setObjectOfInterest(nearest);
        }
      }
    }*/
  }

}

void Environment::addTrafficSign(const ExtendedPose2d &pose, int type){
  TrafficSignPtr sign(new TrafficSign(type, pose));

  if(Broker::isActive()) {
    //KUER MODE
    if(type == MARKER_ID_NOOVERTAKING) {

        double dist = sign->calcDistTo(this->getCar());

        std::string json = "{\"x\": " + boost::lexical_cast<std::string>(sign->getX()) +
            ", \"y\": " + boost::lexical_cast<std::string>(sign->getY()) +
	    ", \"yaw\": " + boost::lexical_cast<std::string>(sign->getYaw()) +
	    ", \"ownX\": " + boost::lexical_cast<std::string>(this->getCar()->getX()) +
	     ", \"ownY\": " + boost::lexical_cast<std::string>(this->getCar()->getY()) +
	     ", \"ownYaw\": " + boost::lexical_cast<std::string>(sign->getYaw()) +
	    ", \"distance\": " + boost::lexical_cast<std::string>(dist)
        	+ "}";
        LOGGING_INFO( worldLogger, "KUER: sending distance:" << dist <<endl);
        Broker::getInstance()->publish( CHANNEL_SEND_DISTANCE,	json);

    } else if(type == MARKER_ID_ONEWAYSTREET) {
        double dist = sign->calcDistTo(this->getCar());

        std::string json = "{\"x\": " + boost::lexical_cast<std::string>(sign->getX()) +
            ", \"y\": " + boost::lexical_cast<std::string>(sign->getY()) +
	    ", \"yaw\": " + boost::lexical_cast<std::string>(sign->getYaw()) +
	    ", \"ownX\": " + boost::lexical_cast<std::string>(this->getCar()->getX()) +
	     ", \"ownY\": " + boost::lexical_cast<std::string>(this->getCar()->getY()) +
	     ", \"ownYaw\": " + boost::lexical_cast<std::string>(sign->getYaw()) +
	    ", \"distance\": " + boost::lexical_cast<std::string>(dist)
        	+ "}";
        LOGGING_INFO( worldLogger, "KUER: sending checkpoint:" << dist <<endl);
        Broker::getInstance()->publish( CHANNEL_SEND_CHECKPOINT,json);
    }
  }

  addTrafficSign( sign );
}
TrafficSignPtrList Environment::getTrafficSigns(){
  return mTrafficSigns;
}


ExtendedPose2dVector Environment::getPastCarPoses(){
  return mPastCarPoses;
}


unsigned int Environment::getCarPoseIndexBeforePatch(unsigned int patchID){
  return mCarPoseIndexBeforePatch[patchID];
}


bool Environment::existsCarPoseIndexBeforePatch(unsigned int patchID){
  return (mCarPoseIndexBeforePatch.find(patchID) != mCarPoseIndexBeforePatch.end());
}

void Environment::eventTimerFired( timerType type, unsigned long timerID )
{
  /*if( type == TIMER_TYPE_REMOVE_OBSTACLES )
  {
    LOGGING_INFO( worldLogger, "Timer fired (Remove Obstacles)"<<endl);
    removeOldObstacles();
  }*/
  if(type == TIMER_TYPE_REMOVE_OLD_OBJECTS)
  {
    LOGGING_INFO( worldLogger, "Timer fired (Remove Old Objects)"<<endl);
    removeOldObjects();
  }
}


/*double Environment::getRecommendedSpeed(){

  //some random consts
  //double max_dist = 1.0;
  //double min_dist = 0.25;

  LOGGING_INFO( worldLogger, "getRecommendedSpeed 1" << endl);
  double maxSpeed = mDriver->getTargetSpeed();
  LOGGING_INFO( worldLogger, "getRecommendedSpeed 2" << endl);

  if(mTrajectory.size() > 2){

                //consider Trajectory Length (if smaller max dist, slow down)
                int trajStart = getNextTrajIndex();
                ExtendedPose2d nextTrajPoint = mTrajectory[trajStart];
                ExtendedPose2d endTrajPoint = mTrajectory.back();

                double dist = nextTrajPoint.distance(endTrajPoint);

                dist = std::max(0.0, dist-min_dist);

                double distScale = dist / (max_dist - min_dist);

                maxSpeed = MIN_SPEED + (maxSpeed - MIN_SPEED) * distScale;


                //TrajectoryCurvature
                //calc relative TrajectoryCurvatureint trajStart = getNextTrajIndex();
                mTrajectory.calculateOrientations();

                double yaw0 = mTrajectory[0].getYaw();
                if(yaw0 != yaw0){ //nan stuff
                        yaw0 = 0.0;
                }
                double yaw1 = 0.0;
                double yawDiff = 0.0;
                for (Trajectory2d::iterator it=mTrajectory.begin()+1 ; it != mTrajectory.end(); it++)
                {
                        ExtendedPose2d curPose = *it;
                        yaw1 = curPose.getYaw();
                        if(yaw1 != yaw1){ //nan stuff
                                yaw1 = 0.0;
                        }
                        yawDiff += std::abs(yaw1 - yaw0);
                        yaw0 = yaw1;

                }

                yawDiff = yawDiff / (mTrajectory.size()-1);
                double yawScale = (M_PI/2 - yawDiff) / M_PI/2;

                maxSpeed = MIN_SPEED + (maxSpeed - MIN_SPEED) * yawScale;

  double maxObstSpeed = getMaxSpeedConsideringObstacles();
  maxSpeed = std::min(maxSpeed, maxObstSpeed);
  LOGGING_INFO( worldLogger, "Recommended Speed (proudly presented to you by the Environment) : " << maxSpeed << endl );

  return maxSpeed;

}*/

cv::Mat Environment::getEnvAsImage( double x, double y, double radius,
                                    float pixelsPerMeter )
{
  // Let the EnvironmentPainter draw this environmnet:
  return mEnvironmentPainter.getEnvAsImage( this,
      x, y, radius, pixelsPerMeter );
}


TrafficSignPtr Environment::getTrafficSignAtPatch(PatchPtr other){

  return other->getCorrespondingTrafficSign(mCar->getPose());

}


void Environment::replaceStreetPatch(PatchPtr replacePatch, PatchPtr newPatch){


  PatchPtrList children = replacePatch->getChildren();

  newPatch->addChildren(children); // now patch has right children

  PatchPtrList::iterator child;
  for( child = children.begin(); child != children.end(); child++ )
  {
    (*child)->addChild( newPatch );
    (*child)->removeChild( replacePatch );
  }

  EventRegionPtrList evRegions = replacePatch->getEventRegions();
  for( EventRegionPtrList::iterator it = evRegions.begin(); it != evRegions.end(); it++ )
  {
    it = mEventRegions.erase( it );
  }

  evRegions = newPatch->getEventRegions();
  for( EventRegionPtrList::iterator it = evRegions.begin(); it != evRegions.end(); it++ )
  {
    mEventRegions.push_back( *it );
  }

  mOpenPatches.remove( replacePatch );
  removePatchFromList( replacePatch, &mStreet );
  removePatchFromList( replacePatch, &mPastCarPatches );
  addStreetPatch(newPatch);
}

bool Environment::isOnParkSignEventRegion()
{
  for( EventRegionPtrList::iterator it = mEventRegions.begin(); it != mEventRegions.end(); it++ ){
    if((*it)->getEventRegionType() == PARKING_SIGN){
      if((*it)->checkOverlap(getCar())){
        return true;
      }
    }
  }


  return false;
}


void Environment::setLane(enumLane lane){
  mLane = lane;
}

enumLane Environment::getLane(){
  return mLane;
}

ExtendedPose2d Environment::getCarPose() { 
  return getCar()->getPose();
}

void Environment::setRememberObstacles( bool remember )
{
  mRememberObstacles = remember;
  if( remember )
  {
    LOGGING_INFO( worldLogger, "Environment will now remember obstacles on our right side." << endl );
  } else {
    LOGGING_INFO( worldLogger, "Environment will no longer remember obstacles." << endl );
  }
}


////checks if a obstacle is relevant (right now i.e. on the trajectory)
bool Environment::isRelevantObstacle(ObstaclePtr obstacle, Trajectory2d& traj, double distOffset ){

  if(obstacle){

    //LOGGING_INFO( worldLogger, "isRelevantObstacle " << endl );
    if(traj.size() <=0){
      return false;
    }

    Trajectory2d extendedTraj;
    extendedTraj.append(traj);

    double yaw = extendedTraj.back().getYaw();

    ExtendedPose2d nextPoint;
    nextPoint.setX(extendedTraj.back().getX() + cos( yaw ) * 1.0 );
    nextPoint.setY(extendedTraj.back().getY() + sin ( yaw ) * 1.0 );
    nextPoint.setYaw(yaw);
    //extendedTraj.push_back(nextPoint);

    //LOGGING_INFO( worldLogger, "Locked @ isRelevantObstacle " << endl );
//    if(obstacle->getProbability() < EXISTENCE_THRES)
//      return false;

    ExtendedPose2d projection;
    double distance;
    std::size_t nearest_pose_index;

    bool before_traj = calculateProjection(extendedTraj,obstacle->getPose(), projection, distance, nearest_pose_index);

    if(before_traj){
      return false;
    }

    distance = std::abs(distance);

    LOGGING_INFO( worldLogger, "Obstacle Distance: " << distance  << endl );

    //CAR_WIDTH/2 + safty offset
    if(distance <= CAR_WIDTH/2+distOffset){
      LOGGING_INFO( worldLogger, "Relevant Obstacle: " << obstacle->getId()  << endl );
      obstacle->setIsRelevantAcc(true);
      return true;
    }
    else{
      return false;
    }

  }
  else{
    return false;
  }

}



bool Environment::isAccOn() const {
  return mAccOn;
}

void Environment::setAccOn(bool AccOn) {
  mAccOn = AccOn;
}

bool Environment::isOverwriteAccMinSpeed() const {
  return mOverwriteAccMinSpeed;
}

void Environment::setOverwriteAccMinSpeed(bool OverwriteAccMinSpeed) {
  mOverwriteAccMinSpeed = OverwriteAccMinSpeed;
}

double Environment::getNewAccMinSpeed() const {
  return mNewAccMinSpeed;
}

void Environment::setNewAccMinSpeed(double NewAccMinSpeed) {
  mNewAccMinSpeed = NewAccMinSpeed;
}


void Environment::insertObstaclePoints(Trajectory2d& traj){

  if(traj.size() <= 2)
    return;

  /*LOGGING_INFO( worldLogger, "Insert Obstacle Points " << endl );
  for (Trajectory2d::iterator it=traj.begin(); it != traj.end(); it++){
    LOGGING_INFO( worldLogger, "\t Yaw: " << (*it).getYaw() << endl );
  }*/

  double offset = CAR_LENGTH-0.1;

  for(ObstaclePtrList::iterator it = mObstacles.begin(); it != mObstacles.end(); it++) {

    ObstaclePtr obst = *it;
    if(isInFrontOfCar(obst,0.40) && isRelevantObstacle(obst, traj)){

      ExtendedPose2d projection;
      double distance;
      std::size_t nearest_pose_index;

      calculateProjection(traj,obst->getPose(), projection, distance, nearest_pose_index);

      projection.setVelocity(-1.0);
      traj.insert(traj.begin()+nearest_pose_index+1, projection);

      if(nearest_pose_index <= 0){
        traj[0].setVelocity(-1.0);
      }
      else {

        //GO back offset = 70cm from the obstacle and add the stop point there
        double dist = std::max(traj[nearest_pose_index - 1].distance(traj[nearest_pose_index]) ,0.0001);
        double x1 = traj[nearest_pose_index - 1].getX() - traj[nearest_pose_index].getX();
        double y1 = traj[nearest_pose_index - 1].getY() - traj[nearest_pose_index].getY();

        //add Point Wherer is Should stop competely
        double xOff = x1/dist*offset;
        double yOff = y1/dist*offset;

        //add point where it should reach 0.15 m/2 since we cant drive with slower speed
        double xOffSlow = x1/dist*(offset+0.05);
        double yOffSlow = y1/dist*(offset+0.05);



        ExtendedPose2d projectionFront;
        double distanceFront;
        std::size_t nearest_pose_index_front;

        //Add 0 speed point
        calculateProjection(traj,ExtendedPose2d(projection.getX()+xOff, projection.getY()+yOff, 0.0), projectionFront, distanceFront, nearest_pose_index_front);
        projectionFront.setVelocity(-1.0);
        traj.insert(traj.begin()+nearest_pose_index_front+1, projectionFront);

        //Add 0.15 speed point
        calculateProjection(traj,ExtendedPose2d(projection.getX()+xOffSlow, projection.getY()+yOffSlow, 0.0), projectionFront, distanceFront, nearest_pose_index_front);
        projectionFront.setVelocity(0.15);
        traj.insert(traj.begin()+nearest_pose_index_front+1, projectionFront);

      }

    }

  }

  traj.calculateOrientations();

  /*LOGGING_INFO( worldLogger, " ==> " << endl );
  for (Trajectory2d::iterator it=traj.begin(); it != traj.end(); it++){
    LOGGING_INFO( worldLogger, "\t Yaw: " << (*it).getYaw() << endl );
  }*/

}

void Environment::interpolateTrajSpeed(Trajectory2d& traj, unsigned int ind1, unsigned int ind2, bool allZero){

  double zero = 0.00;
  if(isOverwriteAccMinSpeed()){
    zero = getNewAccMinSpeed();
  }

  if(traj.size() < ind1 || traj.size() < ind2)
    return;

  if(allZero){
    for (unsigned int i = ind1; i <= ind2 ; ++i) {
      traj[i].setVelocity(zero);
    }
    return;
  }

  LOGGING_INFO( worldLogger, "Speed Interpol start " << ind1 << ", end "  << ind2 << ", zero: " << zero << endl );

  if(ind2-ind1 > 1){

    double startSpeed = std::max(zero,traj[ind1].getVelocity());
    double endSpeed;
    if(isnan(traj[ind2].getVelocity())){
      endSpeed = startSpeed;
    }
    else {
      endSpeed = std::max(zero, traj[ind2].getVelocity());
    }


    for (unsigned int i = ind1+1; i < ind2 ; ++i) {

      double ratio = traj.lengthBetween(ind1,i) /  (traj.lengthBetween(ind1,i)  + traj.lengthBetween(i,ind2));
      traj[i].setVelocity(startSpeed * (1.0 - ratio) + endSpeed * ratio);

    }

    traj[ind2].setVelocity(endSpeed);

  }

}


void Environment::updateTrajectorySpeed(Trajectory2d& traj){

  if(traj.size() > 1){

    double speed0 = traj[0].getVelocity();

    if(isnan(speed0) || speed0 <= 0){
      speed0 = mDriver->getTargetSpeed();
      traj[0].setVelocity(speed0);
    }

    int startIndex = 0;
    int endIndex = traj.size()-1;
    bool hasObstacle = false;
    for (unsigned int i = 1; i < traj.size() ; i++) {

      if(!isnan(traj[i].getVelocity()) ){
        interpolateTrajSpeed(traj,startIndex,i,hasObstacle);
        if(traj[i].getVelocity() < 0){
          hasObstacle = true;
        }
        startIndex = i;
      }

    }

    interpolateTrajSpeed(traj,startIndex,endIndex,hasObstacle);


  }

}


bool Environment::isMultiRelevantObstacle(ObstaclePtr obstacle, double distOffset){

  for(std::vector<Trajectory2d>::iterator it=mMultiTrajectory.trajectories.begin(); it !=  mMultiTrajectory.trajectories.end(); it++) {

    if(isRelevantObstacle(obstacle, *it, distOffset)){
      return true;
    }
  }
  return false;

}


void Environment::resetStreetLane(){
  for ( PatchPtrList::iterator it = mStreet.begin(); it != mStreet.end(); it++ )
  {
    (*it)->setLane(LANE_RIGHT);
    (*it)->setSwitchType(0);
  }
}

/*bool Environment::updateTrajSpeed(Trajectory2d& traj ){

  ObstaclePtrList relevantObst;

  if(traj.size() <= 0)
    return false;

  ExtendedPose2d fstPose = traj.front();
  fstPose.setVelocity(0.0);

  for(ObstaclePtrList::iterator it = mObstacles.begin(); it != mObstacles.end(); it++) {
    ObstaclePtr obstacle = *it;
    if(isRelevantObstacle(fstPose, obstacle, traj)){
      relevantObst.push_back(obstacle);
    }
  }

  ObstaclePtr lastRelevantObst = getNearstRelevantObstacle(traj.front(), relevantObst, traj);
  double targetSpeed = mDriver->getTargetSpeed();

  for (Trajectory2d::iterator it=traj.begin(); it != traj.end(); it++)
  {
    ExtendedPose2d curPose = *it;
    double speed = targetSpeed;

    if(!isRelevantObstacle(curPose,lastRelevantObst,traj)){
      lastRelevantObst = getNearstRelevantObstacle(curPose, relevantObst, traj);
    }

    if(lastRelevantObst) {
      double dist = lastRelevantObst->getPose().distance(curPose) - CAR_WIDTH;
      speed = std::min(speed, lastRelevantObst->getMaxSpeed(dist));
    }

    speed = std::max(speed , 0.0);

    (*it).setVelocity(speed);


  }

//  for (Trajectory2d::iterator it2=traj.begin(); it2 != traj.end(); it2++){
//    LOGGING_INFO( worldLogger, "Speed =P: " << (*it2).getVelocity()  << endl );
//  }

  return true;
}*/

void Environment::updateMultiTrajSpeed(){

  if( mMultiTrajectory.trajectories.size() == 0 )
    return;

  for(std::vector<Trajectory2d>::iterator it=mMultiTrajectory.trajectories.begin(); it !=  mMultiTrajectory.trajectories.end(); it++){
    if(isAccOn()) {
      insertObstaclePoints(*it);
    }
    updateTrajectorySpeed(*it);

  }

}


bool Environment::calculateProjection(const Trajectory2d& trajectory, const ExtendedPose2d& curPosition, ExtendedPose2d& projection,
                                            double& distance, std::size_t& nearest_pose_index)
{
  if( trajectory.size() == 0 )
  {
    return false;
  }
  float dist1, dist2, ratio, curr_distance_squared;
  float t;

  float shortest_distance_squared = std::numeric_limits<float>::infinity();

  bool shortest_distance_found_before_first_point = true;
  nearest_pose_index = 0;

  Position2d position = curPosition.getPosition();
  const double& xP = position.x();
  const double& yP = position.y();
  for( u_int32_t i = 0; i < trajectory.size()-1 ; ++i)
  {
    const double& x1 = trajectory[i].getX();
    const double& y1 = trajectory[i].getY();
    const double& x2 = trajectory[i+1].getX();
    const double& y2 = trajectory[i+1].getY();

    double Xnew, Ynew;

    const double& APx = xP - x1;
    const double& APy = yP - y1;
    const double& ABx = x2 - x1;
    const double& ABy = y2 - y1;
    const double& magAB2 = ABx*ABx + ABy*ABy;
    const double& ABdotAP = ABx*APx + ABy*APy;
    t = ABdotAP / magAB2;

    if ( t < 0)
    {
      Xnew = x1;
      Ynew = y1;
    }
    else if (t > 1)
    {
      Xnew = x2;
      Ynew = y2;
    }
    else
    {
      Xnew = x1 + ABx*t;
      Ynew = y1 + ABy*t;
    }


    double a = (xP - Xnew)*(xP - Xnew);
    double b = (yP - Ynew)*(yP - Ynew);
    curr_distance_squared = (a + b);

    if (curr_distance_squared < shortest_distance_squared)
    {
      shortest_distance_squared = curr_distance_squared;
      shortest_distance_found_before_first_point = (i == 0 && t < 0);

      nearest_pose_index = i;

      dist1 = sqrt((Xnew - x1)*(Xnew - x1) + (Ynew - y1)*(Ynew - y1));
      dist2 = sqrt((Xnew - x2)*(Xnew - x2) + (Ynew - y2)*(Ynew - y2));
      ratio = dist1/(dist1+dist2);

    }
  }
  //m_ratio = ratio;

  // A-B:
  const Position2d vector_ab = trajectory[nearest_pose_index].getPose().rotation() * Position2d(1.0, 0.0);
  // A-Vehicle
  const Position2d vector_a_vehicle = position - trajectory[nearest_pose_index].getPosition();

  // pose of the car is before the beginning of the trajectory -> use distance to projection on AB-Vector
  if (shortest_distance_found_before_first_point)
  {
    // copy speed and curvature from first point
    projection = trajectory.front();

    // Projection
    const double t = vector_a_vehicle.dot(vector_ab)/vector_ab.squaredNorm();
    projection.setPosition(trajectory[0].getPosition() + vector_ab * t);

    // Calculate distance
    distance = (projection.getPosition() - position).norm();

  }
  else
  {
    //calculate projected pose
    projection = oadrive::core::Interpolator::interpolateLinear(trajectory[nearest_pose_index], trajectory[nearest_pose_index+1], ratio);

    // distance calculation
    distance = sqrt(shortest_distance_squared);

  }



  //Sign of distance
  Eigen::Matrix<double, 2, 2> m;
  m.col(0) <<vector_ab;
  m.col(1) <<vector_a_vehicle;

  // If the determinant is negative, the point lies on the right hand to the line
  if (m.determinant() > 0.0)
    distance = -distance;

  return shortest_distance_found_before_first_point;
}


bool Environment::isInFrontOf(ExtendedPose2d firstPose, ExtendedPose2d other, double offset){


  ExtendedPose2d carOffsetPose = firstPose;

  double yaw = firstPose.getYaw();

  carOffsetPose.setX(carOffsetPose.getX()+cos( yaw )*offset);
  carOffsetPose.setY(carOffsetPose.getY()+sin( yaw )*offset);

  double x1 = cos(yaw);
  double y1 = sin(yaw);

  double x2 = other.getX() - carOffsetPose.getX();
  double y2 = other.getY() - carOffsetPose.getY();

  double atx = (x1*x2 + y1*y2);

  if(atx < 0){
    // 		LOGGING_INFO( worldLogger, "angle is greater than PI/2 ->back : " << endl );
    return false;
  }
  else{
    // 		LOGGING_INFO( worldLogger, "angle is lower than PI/2 -> front : " << endl );
    return true;
  }
}

bool Environment::isInFrontOfCar(EnvObjectPtr other, double offset){

  if(!other)
    return false;



  ExtendedPose2d carPose = getCar()->getPose();
  ExtendedPose2d otherPose = other->getPose();


  LOGGING_INFO( worldLogger, "In Front of Car : " << isInFrontOf(carPose,otherPose,offset) << endl );

  return isInFrontOf(carPose,otherPose,offset);

}


void Environment::getObstacleInTrajectory(ObstaclePtrList& inTrajObst, double distOffset)
{
  for(ObstaclePtrList::iterator it = mObstacles.begin();it != mObstacles.end(); it++){
    ObstaclePtr obstacle = *it;
    if(isInFrontOfCar(obstacle) && isMultiRelevantObstacle(obstacle, distOffset)){
      inTrajObst.push_back(obstacle);
    }
  }
}
/////////////////////////////////////////////////////////
// Functions currently not used. Re-Add only if you need them, please!

/*void Environment::removeToNearToSignObstacles(){
  for(ObstaclePtrList::iterator it = mObstacles.begin();it != mObstacles.end();){
    ObstaclePtr obstacle = *it;
    if(obstacle->getSawTrafficSign() <= 0 && isToNearToSign(*it)){
      EventRegionPtrList *evRegions = obstacle->getEventRegions();
      for(EventRegionPtrList::iterator itEventRegions = evRegions->begin();itEventRegions != evRegions->end();)
      {
        removeEventRegion((*itEventRegions));
        itEventRegions++;
      }
      it = mObstacles.erase( it );
    } else {
      it++;
    }
  }
}*/

/*bool Environment::isToNearToSign(ObstaclePtr obstacle){

  TrafficSignPtrList::iterator signIt;
  for( signIt = mTrafficSigns.begin(); signIt != mTrafficSigns.end(); signIt ++ )
  {
    if(obstacle->getPose().distance((*signIt)->getPose()) < OBSTACLE_SIGN_DISTANCE){
      return true;
    }
  }
  return false;
}*/

/*ObstaclePtrList Environment::getObstaclesCrossEnvObject(EnvObjectPtr other)
{
  ObstaclePtrList crossingObstacles;

  for(ObstaclePtrList::iterator it = mObstacles.begin();it != mObstacles.end();it++)
  {
    ObstaclePtr obstacle = *it;
    if(obstacle->checkOverlap(other))
    {
      crossingObstacles.push_back(obstacle);
    }
  }
  return crossingObstacles;
}*/


/*ObstaclePtr Environment::getObstacleInTrajectory()
{
  ObstaclePtr inTrajObst;

  for(ObstaclePtrList::reverse_iterator it = mObstacles.rbegin();it != mObstacles.rend(); it++){
    ObstaclePtr obstacle = *it;
    if(isRelevantObstacle(obstacle)){
      inTrajObst = obstacle;
      return inTrajObst;
    }
  }

  return inTrajObst;
}*/


/*ObstaclePtr Environment::getNearestObstacle(const ExtendedPose2d &pose)
{
  // If empty, return NULL pointer:
  if( mObstacles.size() == 0 )
    return ObstaclePtr();

  ObstaclePtrList::iterator it;

  for( it = mObstacles.begin(); it != mObstacles.end(); it++ )
  {
    (*it)->calcTempDist( pose );
  }

  mObstacles.sort(compareObstacles);

  return mObstacles.front();
}*/


/*void Environment::removeOldObstacles(){

  LOGGING_INFO( worldLogger, "Locked @ removeOldObstacles " << endl );

  int time = std::time(0);
  for(ObstaclePtrList::iterator it = mObstacles.begin();it != mObstacles.end();){
    ObstaclePtr obstacle = *it;
    if(time - obstacle->getTime() > obstacle->getTimeToLive()){
      EventRegionPtrList *evRegions = obstacle->getEventRegions();
      for(EventRegionPtrList::iterator itEventRegions = evRegions->begin();itEventRegions != evRegions->end();)
      {
        removeEventRegion((*itEventRegions));
        itEventRegions++;
      }
      it = mObstacles.erase( it );
    } else {
      it++;
    }
  }

  if(mTimer != NULL)
    mTimer->setTimer( 1000, TIMER_TYPE_REMOVE_OBSTACLES );
}*/

/*double Environment::getMaxSpeedConsideringObstacles(){

  double speed = 1000000;

  for(ObstaclePtrList::iterator it = mObstacles.begin();it != mObstacles.end(); it++){
    ObstaclePtr obstacle = *it;
    if(obstacle->getSawTrafficSign() <= 0 && isRelevantObstacle(obstacle)){
      obstacle->setIsRelevantAcc(true);  //for debugging
      double dist = getCar()->calcDistTo(obstacle) - CAR_WIDTH;
      double maxSpeed = obstacle->getMaxSpeed(dist);
      if(dist < OBSTACLE_FULL_BREAK_THRES){
        maxSpeed = 0;
      }
      if(maxSpeed < speed){
        speed = maxSpeed;
      }

    }
    else
    {
      obstacle->setIsRelevantAcc(false);  //for debugging
    }

  }


  LOGGING_INFO( worldLogger, "ACC Speed : " << speed << endl );

  return speed;
}*/

/*bool Environment::obstacleStillExistsAndIsRelevant(ObstaclePtr obstacle){

  if(!obstacle)
    return false;

  bool exists = false;
  for(ObstaclePtrList::iterator it = mObstacles.begin();it != mObstacles.end(); it++){
    if((*it)->getId() == obstacle->getId()){
      exists = true;
    }
  }

  if(!exists)
    return false;

  return isRelevantObstacle(obstacle);
}*/


/*bool Environment::isInFrontOfCar(EnvObjectPtr other, double offset){

  if(!other)
    return false;

  ExtendedPose2d carOffsetPose = getCar()->getPose();

  double yaw = getCar()->getPose().getYaw();

  carOffsetPose.setX(carOffsetPose.getX()+cos( yaw )*offset);
  carOffsetPose.setY(carOffsetPose.getY()+sin( yaw )*offset);

  double x1 = cos(yaw);
  double y1 = sin(yaw);

  double x2 = other->getPose().getX() - carOffsetPose.getX();
  double y2 = other->getPose().getY() - carOffsetPose.getY();

  // Ugly:
  double denom = std::max( sqrt( x2*x2 + y2*y2 ), 0.000001 );

  x2 = x2 / denom;
  y2 = y2 / denom;

  double angle = acos(x1*x2 + y1*y2);

  // 	LOGGING_INFO( worldLogger, "angle: " << angle*180/M_PI << endl );

  if(angle > M_PI_2){
    // 		LOGGING_INFO( worldLogger, "angle is greater than PI/2 ->back : " << endl );
    return false;
  }
  else{
    // 		LOGGING_INFO( worldLogger, "angle is lower than PI/2 -> front : " << endl );
    return true;
  }
}*/

/*void Environment::deleteObstaclesInFrontOfCar(double offset){

  for(ObstaclePtrList::iterator it = mObstacles.begin();it != mObstacles.end(); it++){

    ObstaclePtr obstacle = *it;
    // 		LOGGING_INFO( worldLogger, "Delete?: " << obstacle->getId() << endl );
    if( !obstacle->isDoNotDelete() && isInFrontOfCar(obstacle, offset) ){
      //             LOGGING_INFO( worldLogger, "Deleted : " << obstacle->getId() << endl );

      EventRegionPtrList *evRegions = obstacle->getEventRegions();
      for(EventRegionPtrList::iterator itEventRegions = evRegions->begin();itEventRegions != evRegions->end();)
      {
        removeEventRegion((*itEventRegions));
        itEventRegions++;
      }
      it = mObstacles.erase( it );
    }
  }
}*/

////checks if a obstacle is relevant (right now i.e. on the trajectory)
/*bool Environment::isRelevantObstacle(ObstaclePtr obstacle){


  if(obstacle){
    
    LOGGING_INFO( worldLogger, "isRelevantObstacle " << endl );
    if(mTrajectory.size() <=0){
      return false;
    }
    

    LOGGING_INFO( worldLogger, "Locked @ isRelevantObstacle " << endl );

    if(obstacle->getProbability() < EXISTENCE_THRES)
      return false;

    bool relevant = false;
    LOGGING_INFO( worldLogger, "isRelevantObstacle 1" << endl );
    int trajStart = getNextTrajIndex();
    LOGGING_INFO( worldLogger, "isRelevantObstacle 2" << endl );

    bool begin = true;
    ExtendedPose2d prevPose(0.1,0.1,0.1);

    double dist;
    double DIST_OFF = 0.2;


    for (Trajectory2d::iterator it=mTrajectory.begin() + trajStart ; it != mTrajectory.end(); it++)
    {
      ExtendedPose2d curPose = *it;
      dist = curPose.distance(prevPose);
      if(dist > DIST_OFF || begin){

        EnvObjectPtr car(new EnvObject(curPose));
        if(obstacle->checkOverlap(car)){
          relevant = true;
          break;
        }
        prevPose = curPose;
        begin = false;

      }

    }


    LOGGING_INFO( worldLogger, "isRelevantObstacle end" << endl );

    return relevant;
    
  }
  else{
    return false;
  }

}*/


/*bool Environment::checkRelevantObstacleAtLine(ObstaclePtr obstacle, const ExtendedPose2d &curPose, const ExtendedPose2d &nextPose)
{
  bool relevant = false;
  ExtendedPose2d midPoint;
  midPoint.setX((nextPose.getPosition().x()-curPose.getPosition().x())*0.5+curPose.getPosition().x());
  midPoint.setY( (nextPose.getPosition().y()-curPose.getPosition().y())*0.5+curPose.getPosition().y());
  midPoint.setYaw(curPose.getYaw());
  double length = (nextPose.getPosition()-curPose.getPosition()).norm();
  EnvObjectPtr corridor(new EnvObject (midPoint,CAR_WIDTH,length ));
  if(obstacle->checkOverlap(corridor)){
    relevant = true;
    //LOGGING_INFO(worldLogger,"Found relevant Obstacle");
  }
  return relevant;

}*/

std::string Environment::toJson(double radius) {

  std::stringstream ss;
  ss << std::setprecision( 2 );
  ss << std::fixed;

  ss << ""
    << "{";

  //the car
  EnvObjectPtr car = getCar();
  ss << "\"car\": " <<
    "{" <<
    "\"type\": \"car\"," <<
    "\"x\": " << car->getX() << "," <<
    "\"y\": " << car->getY() <<  "," <<
    "\"yaw\": " << car->getYaw() <<
    "},";


  //Patches
  ss << "\"patches\": [";

  PatchPtrList::iterator lastElement = mStreet.end();
  --lastElement;
  for ( PatchPtrList::iterator it = mStreet.begin(); it != mStreet.end(); it++ )
  {
      PatchPtr p = *it;
      ss << p->toJson();

      if (it != lastElement) {
        ss << ",";
      }

  }
  for ( PatchPtrList::iterator it = mParkingLots.begin(); it != mParkingLots.end(); it++ ) {
      ss << ",";
      PatchPtr p = *it;
      ss << p->toJson();
  }

  ss << "]";


  //end
  ss << ""
      "}";

  return ss.str();

}

void Environment::setCurrentUSSensorLimits( oadrive::obstacle::enumUSSensorLimits state )
{
  mCurrentUSSensorLimits = state;
  usSensor* limits = getCurrentUSSensorLimits();

  LOGGING_INFO( worldLogger, "Set US sensor limits to:" << endl
      << "\tfrontLeft: " << limits->frontLeft << endl
      << "\tfrontCenterLeft: " << limits->frontCenterLeft << endl
      << "\tfrontCenter: " << limits->frontCenter << endl
      << "\tfrontCenterRight: " << limits->frontCenterRight << endl
      << "\tfrontRight: " << limits->frontRight << endl
      << "\tsideLeft: " << limits->sideLeft << endl
      << "\tsideRight: " << limits->sideRight << endl
      << "\trearLeft: " << limits->rearLeft << endl
      << "\trearCenter: " << limits->rearCenter << endl
      << "\trearRight: " << limits->rearRight << endl);
}

void Environment::initUSSensorLimits()
{
  // Read the limits of the sensors for the various states.
  // Note: Negative values deactivate a sensor entirely.
  usSensor limits;
  limits.frontLeft = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_FRONTLEFT", -1.0);
  limits.frontCenterLeft = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_FRONTCENTERLEFT", -1.0);
  limits.frontCenter = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_FRONTCENTER", -1.0);
  limits.frontCenterRight = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_FRONTCENTERRIGHT", -1.0);
  limits.frontRight = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_FRONTRIGHT", -1.0);
  limits.sideLeft = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_SIDELEFT", -1.0);
  limits.sideRight = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_SIDERIGHT", -1.0);
  limits.rearLeft = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_REARLEFT", -1.0);
  limits.rearCenter = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_REARCENTER", -1.0);
  limits.rearRight = Config::getDouble( "USSensors", "LIMIT_FOR_CROSSING_REARRIGHT", -1.0);
  mUSSensorLimits.insert( std::pair<enumUSSensorLimits, usSensor>( LIMIT_FOR_CROSSING, limits ));

  limits.frontLeft = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_FRONTLEFT", -1.0);
  limits.frontCenterLeft = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_FRONTCENTERLEFT", -1.0);
  limits.frontCenter = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_FRONTCENTER", -1.0);
  limits.frontCenterRight = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_FRONTCENTERRIGHT", -1.0);
  limits.frontRight = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_FRONTRIGHT", -1.0);
  limits.sideLeft = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_SIDELEFT", -1.0);
  limits.sideRight = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_SIDERIGHT", -1.0);
  limits.rearLeft = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_REARLEFT", -1.0);
  limits.rearCenter = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_REARCENTER", -1.0);
  limits.rearRight = Config::getDouble( "USSensors", "LIMIT_FOR_SEARCH_PARKING_REARRIGHT", -1.0);
  mUSSensorLimits.insert( std::pair<enumUSSensorLimits, usSensor>( LIMIT_FOR_SEARCH_PARKING, limits ));

  limits.frontLeft = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_FRONTLEFT", -1.0);
  limits.frontCenterLeft = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_FRONTCENTERLEFT", -1.0);
  limits.frontCenter = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_FRONTCENTER", -1.0);
  limits.frontCenterRight = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_FRONTCENTERRIGHT", -1.0);
  limits.frontRight = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_FRONTRIGHT", -1.0);
  limits.sideLeft = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_SIDELEFT", -1.0);
  limits.sideRight = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_SIDERIGHT", -1.0);
  limits.rearLeft = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_REARLEFT", -1.0);
  limits.rearCenter = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_REARCENTER", -1.0);
  limits.rearRight = Config::getDouble( "USSensors", "LIMIT_FOR_OVERTAKING_REARRIGHT", -1.0);
  mUSSensorLimits.insert( std::pair<enumUSSensorLimits, usSensor>( LIMIT_FOR_OVERTAKING, limits ));

  limits.frontLeft = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_FRONTLEFT", -1.0);
  limits.frontCenterLeft = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_FRONTCENTERLEFT", -1.0);
  limits.frontCenter = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_FRONTCENTER", -1.0);
  limits.frontCenterRight = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_FRONTCENTERRIGHT", -1.0);
  limits.frontRight = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_FRONTRIGHT", -1.0);
  limits.sideLeft = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_SIDELEFT", -1.0);
  limits.sideRight = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_SIDERIGHT", -1.0);
  limits.rearLeft = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_REARLEFT", -1.0);
  limits.rearCenter = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_REARCENTER", -1.0);
  limits.rearRight = Config::getDouble( "USSensors", "LIMIT_FOR_DRIVING_REARRIGHT", -1.0);
  mUSSensorLimits.insert( std::pair<enumUSSensorLimits, usSensor>( LIMIT_FOR_DRIVING, limits ));

  limits.frontLeft = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_FRONTLEFT", 1.0);
  limits.frontCenterLeft = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_FRONTCENTERLEFT", 1.0);
  limits.frontCenter = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_FRONTCENTER", 1.0);
  limits.frontCenterRight = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_FRONTCENTERRIGHT", 1.0);
  limits.frontRight = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_FRONTRIGHT", 1.0);
  limits.sideLeft = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_SIDELEFT", -1.0);
  limits.sideRight = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_SIDERIGHT", -1.0);
  limits.rearLeft = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_REARLEFT", -1.0);
  limits.rearCenter = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_REARCENTER", -1.0);
  limits.rearRight = Config::getDouble( "USSensors", "LIMIT_FOR_PED_CROSSING_REARRIGHT", -1.0);
  mUSSensorLimits.insert( std::pair<enumUSSensorLimits, usSensor>( LIMIT_FOR_PED_CROSSING, limits ));

  limits.frontLeft = -1.0;
  limits.frontCenterLeft = -1.0;
  limits.frontCenter = -1.0;
  limits.frontCenterRight = -1.0;
  limits.frontRight = -1.0;
  limits.sideLeft = -1.0;
  limits.sideRight = -1.0;
  limits.rearLeft = -1.0;
  limits.rearCenter = -1.0;
  limits.rearRight = -1.0;
  mUSSensorLimits.insert( std::pair<enumUSSensorLimits, usSensor>( LIMIT_DEACTIVATE, limits ));
}


}	// namespace
}	// namespace

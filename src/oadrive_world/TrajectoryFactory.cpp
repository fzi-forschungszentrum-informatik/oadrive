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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-16
 *
 */
//----------------------------------------------------------------------

#include "TrajectoryFactory.h"
#include <oadrive_core/Interpolator.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <sstream>
#include <oadrive_world/worldLogging.h>

#include "TrajectoryDatabase.h"
#include "Environment.h"

using namespace oadrive::core;
using namespace oadrive::control;
using namespace boost::filesystem;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive{
namespace world{

TrajectoryFactory::TrajectoryFactory()
  : mTrajectoryMode( PATCH_TRAJECTORY )
  , mMaxCurvature(1.09)
  , mDebugMode( false )
  , mTrajectoryCounter( 0 )
{
}

void TrajectoryFactory::setFixedTrajectory( MultiTrajectory traj )
{
  //if( smooth )
    //Interpolator::smoothBSpline( traj, 2 );

  // Check if trajectory is long enough?
  // Must be at least 20 points long.
  /*if( traj.size() < 5 )
        {
                LOGGING_INFO( worldLogger, "Trajectory direction: " << traj.isForwardTrajectory() << endl );
                //extrapolateFront( traj );
        }*/

  mTrajectoryMode = FIXED_TRAJECTORY;
  Environment::getInstance()->setTrajectory( traj );
  //std::cout << "[TrajectoryFactory] Set fixed trajectory." << std::endl;
}

void TrajectoryFactory::setFixedTrajectory( enumTrajectory trajName, double scaleFactor  )
{
  bool appendToCar = true;

  mMultiCounter = 0;

  switch( trajName )
  {
    case TRAJ_PULLOUT_PARALLEL:
      mTrajectoryName = "pullout_parallel";
      break;
    case TRAJ_PARKING_PARALLEL:
      //appendToCar = false;	// append to patch, not car!
      mTrajectoryName = "park_parallel";
      break;
    case TRAJ_PARKING_CROSS:
      mTrajectoryName = "park_cross";
      break;
    case TRAJ_PULLOUT_CROSS_LEFT:
      mTrajectoryName = "pullout_cross_left";
      break;
    case TRAJ_PULLOUT_CROSS_RIGHT:
      mTrajectoryName = "pullout_cross_right";
      break;
    case TRAJ_BACKUP_OBSTACLE:
      mTrajectoryName = "backup_obstacle";
      break;
    case TRAJ_FORWARD_SMALL:
      mTrajectoryName = "forward_small";
      break;
    case TRAJ_U_TURN:
      mTrajectoryName = "turn_slow";
      break;
    case TRAJ_MIDDLE:
      mTrajectoryName = "middle";
      break;
    default:
      mTrajectoryName = "";
      break;
  }

  LOGGING_INFO( worldLogger, "Trajectory name: " << mTrajectoryName << endl );

  if( TrajectoryDatabase::hasTrajectory( mTrajectoryName ) )
  {
    MultiTrajectory multiTraj = TrajectoryDatabase::getTrajectory( mTrajectoryName );
    LOGGING_INFO( worldLogger, "\tTrajectory size: " << multiTraj.trajectories.size() << endl );
    if(scaleFactor > 0){
      LOGGING_INFO( worldLogger, "\tHey Ho scale traj with : "  << scaleFactor << endl );
      for(std::vector<Trajectory2d>::iterator it=multiTraj.trajectories.begin(); it !=  multiTraj.trajectories.end(); it++) {
        for(unsigned int i = 0; i < (*it).size(); ++i) {
          (*it)[i].setY((*it)[i].getY() * scaleFactor);
          (*it)[i].setX((*it)[i].getX() * scaleFactor);
        }
      }
    }
    if( appendToCar ) {
      multiTraj =
        rotateAndMoveTrajectory( multiTraj, Environment::getInstance()->getCarPose() );
    }
    LOGGING_INFO( worldLogger,
        "\tTrajectory size after rotateAndMove: " << multiTraj.trajectories.size() << endl );
    //setFixedTrajectory( multiTraj.trajectories[0] );
    setFixedTrajectory( multiTraj );
    mCurrentMultiTraj = multiTraj;
    LOGGING_INFO( worldLogger, "\tTrajectory size after copy: " <<
        mCurrentMultiTraj.trajectories.size() << endl );
  } else {
    std::stringstream sstr;
    sstr << "Cannot find trajectory in database for manuever " << trajName << ".";
    throw( sstr.str().c_str() );
  }
}

void TrajectoryFactory::reset()
{
}

void TrajectoryFactory::setGenerateFromPatches()
{
  // Remember that we are currently in PATCH_TRAJECTORY mode:
  mTrajectoryMode = PATCH_TRAJECTORY;
}

bool TrajectoryFactory::generateFromPatches(bool useOld)
{
  //std::cout << "[TrajectoryFactory] Generating patch trajectory." << std::endl;

  LOGGING_INFO( worldLogger, "[TF] Generating patch trajectory." << endl );

  //new trajectory
  Trajectory2d traj;

  if( mDebugMode )
  {
    mTrajectoryCounter ++;
  }

  //setInitialPose( mEnvironment->getCarPose() );
  //std::cout << "[TrajectoryFactory] Warning: no initial pose for patch trajectory." <<
  //std::endl << "[TrajectoryFactory] Using car pose." << std::endl;
  //LOGGING_INFO( worldLogger, "Setting initial pose to car pose!" << endl );

  PatchPtrList* street = Environment::getInstance()->getStreet();

//  LOGGING_INFO( worldLogger, "[TF] Patches in street: " << street->size() << endl );

  if( street->size() > 0 )
  {
    PatchPtr iterator = *(street->begin());
    PatchPtrList visited;

    ExtendedPose2d lastPose = Environment::getInstance()->getCarPose();
    ExtendedPose2d lastPatchCenterPose = Environment::getInstance()->getCarPose();

    PatchPtrList* pastCarPatches = Environment::getInstance()->getPastCarPatches();

    ExtendedPose2dVector pastCarPoses = Environment::getInstance()->getPastCarPoses();

    //int appendTrajSize = std::min(20,(int)pastCarPoses.size());

    int lastUsedPastPose = pastCarPoses.size() - 1;

//    LOGGING_INFO( worldLogger, "[TF] Number of pastCarPatches: " << pastCarPatches->size() << endl );
//    LOGGING_INFO( worldLogger, "[TF] Iterator: " << iterator->getId() << " " << iterator->getPose() << endl );

    if(Environment::getInstance()->isCarHasBackedUp()){

//      LOGGING_INFO( worldLogger, "[TF] Backed up." << endl );
      //case reverted
      if(pastCarPatches->size() > 0 && pastCarPatches->back())
        iterator = pastCarPatches->back();
      //LOGGING_INFO( worldLogger, "[TF] Revered CASE!" << endl );
      lastPose = Environment::getInstance()->getCarPose();
      lastPatchCenterPose = iterator->getPose();
      PatchPtr next = iterator->getChild(Environment::getInstance()->getCar()->getPose().getYaw(), DD_STRAIGHT);
      if(next){
        iterator = next;
      }
      lastUsedPastPose = 0;

    }
    // If we've already been on a patch, start there:
    else if(pastCarPatches->size() >= 3)
    {

//      LOGGING_INFO( worldLogger, "[TF] PastCarPatches >= 3" << endl );

      PatchPtrList::iterator it = pastCarPatches->begin();
      std::advance(it, pastCarPatches->size()-3);
      iterator = *it;
      unsigned int poseIndexBeforeLastPatch = Environment::getInstance()->getCarPoseIndexBeforePatch(iterator->getId());

      lastPose = pastCarPoses[poseIndexBeforeLastPatch];
      lastPatchCenterPose = (iterator)->getPose();

//      LOGGING_INFO( worldLogger, "[TF] Iterator: " << iterator->getId() << " " << iterator->getPose() << endl );
      //LOGGING_INFO( worldLogger, "Looking for child for " << iterator->getId() << endl );
      //LOGGING_INFO( worldLogger, " yaw: " << lastPose.getYaw() << endl );
      PatchPtr next = iterator->getNextChild( lastPose.getYaw() );
      //PatchPtr next = *(it++);

      if( next )
        iterator = next;
      else
//        LOGGING_WARNING( worldLogger, "No child patch found!" << endl );

     // LOGGING_INFO( worldLogger, "[TF] Next: " << next->getId() << " " << next->getPose() << endl );

      lastUsedPastPose = Environment::getInstance()->getCarPoseIndexBeforePatch( iterator->getId() );
      lastUsedPastPose = -1;

      //appendTrajSize = std::min(appendTrajSize, poseIndexBeforeLastPatch);

    }
    else if(pastCarPatches->size() == 1 && pastCarPatches->size() == 2){

//      LOGGING_INFO( worldLogger, "[TF] PastCarPatches 1,2" << endl );

      iterator = *(street->begin());
      PatchPtr next;
      int poseIndexBeforeLastPatch = Environment::getInstance()->getCarPoseIndexBeforePatch(iterator->getId());

      //std::cout << "Pose index: " << poseIndexBeforeLastPatch << std::endl;

      lastPose = pastCarPoses[poseIndexBeforeLastPatch];
      lastPatchCenterPose = iterator->getPose();
      next = iterator->getNextChild(lastPose.getYaw());
      if(next){
        iterator = next;
      }
      else{
        lastPatchCenterPose = pastCarPoses[poseIndexBeforeLastPatch];
      }
      lastUsedPastPose = Environment::getInstance()->getCarPoseIndexBeforePatch( iterator->getId() );




      //appendTrajSize = std::min(appendTrajSize, poseIndexBeforeLastPatch);
    }
    else{

//      LOGGING_INFO( worldLogger, "[TF] PastCarPatches 0" << endl );
      lastPose = pastCarPoses.back();
      iterator = *(street->begin());
      lastPatchCenterPose = iterator->getPose();

      PatchPtr next = iterator->getNextChild( lastPose.getYaw() );

      if(next){
        iterator = next;
      }
      else{
        lastPatchCenterPose = pastCarPoses.back();
      }
    }

    //append previous driven trajectory

    if(lastUsedPastPose == 0){
      lastUsedPastPose = pastCarPoses.size() - 1;
    }

    unsigned int firstUsedPastPose = 0;
    if( lastUsedPastPose > 5 )
      firstUsedPastPose = lastUsedPastPose - 5;

    //std::cout << "first: " << firstUsedPastPose << " last: " << lastUsedPastPose << std::endl;

    if(lastUsedPastPose != -1) {
      for (ExtendedPose2dVector::iterator it = pastCarPoses.begin() + firstUsedPastPose;
           it != pastCarPoses.begin() + lastUsedPastPose + 1; it++) {
        traj.push_back(*it);
      }
    }

    //Dont start every time from the beginnig, but start from past Patches
    /*if(pastCarPatches->size() >= GO_BACK_IN_TRAJ_HISTORY){
                  PatchPtrList::iterator l_front = pastCarPatches->begin();
                  std::advance(l_front, pastCarPatches->size()-GO_BACK_IN_TRAJ_HISTORY);

                  PatchPtr startPatch = *l_front;

                  lastPose = carPosesOnPatch->at(startPatch->getId()).back();
                  lastPatchCenterPose = startPatch->getPose();

                  std::advance(l_front, 1);

                  iterator = *l_front;
                  }*/

    PatchPtr lastPastPatch;
    if( pastCarPatches->size() > 0 )
    {
      lastPastPatch = pastCarPatches->back();
    }
    if( lastPastPatch )
    {
//      LOGGING_INFO( worldLogger, "lastPastPatch: " << lastPastPatch->getId() << endl );
    }

    int counter = 0;
    bool thisPatchShouldBeFixed = false;

    while( iterator && counter < GO_FRONT_PATCHES )
    {
      //std::cout << "[Environment]: getTrajectory() ID:" << iterator->getId() << std::endl;
      visited.push_back( iterator );
      LOGGING_INFO( worldLogger, "[TF] Loop Iterator: " << iterator->getId() << " " << iterator->getPose() << endl );

      // Find out which child of the current patch (==iterator) will be the next
      // patch, depending on the last patches' position. It also takes into account
      // the driving direction which was set for the current patch.
      // If the last patch was, for example, to the NORTH of the current patch, then
      // getNextChild will return the child towards the SOUTH (but only if the
      // drivingDirection set for the iterator is "DD_STRAIGHT"!)
      PatchPtr next = iterator->getNextChild( lastPatchCenterPose );
      /*LOGGING_INFO( worldLogger, "\tgetTrajectory next: " << next << endl );
                        if( next )
                        {
                                LOGGING_INFO( worldLogger, "\t\tnext ID: " << next->getId() << endl );
                                LOGGING_INFO( worldLogger, "\t\tnext Type: " << next->getPatchType() << endl );
                        }*/
      if(next)
      {
//        LOGGING_INFO( worldLogger, "[TF] Next: " << next->getId() << " " << next->getPose() << endl );
        if(iterator->isSwitchPatch())
        {
//          LOGGING_INFO( worldLogger, "\t\tPatch is a Switch Patch: " << iterator->getId() << endl );
//          LOGGING_INFO( worldLogger, "\t\tSwitched Lane at Patch: " << next->getId() << endl );
          if(iterator->getSwitchType() == 1)
          {
            next->setLane( LANE_LEFT );
          } else if(iterator->getSwitchType() == 2) {
            next->setLane( LANE_RIGHT );
          }
        } else {
          //LOGGING_INFO( worldLogger, "\t\tSwitched NOT Lane at Patch: " << next->getId() << endl );
          next->setLane(iterator->getLane());
        }
      }

      //get the trajectory reference points of the patch and append it to current trajectory:
      Trajectory2d* t = iterator->getTrajectoryFromMC( lastPose, iterator->getLane() );
//      LOGGING_INFO( worldLogger, "[TF]\tSize of traj part: " << t->size() << endl );
//      LOGGING_INFO( worldLogger, "[TF] Iterator patch type: " << iterator->getPatchType() << endl );

      if( t->size() == 0 ) break;

      if( iterator->getPatchType() == CROSS_SECTION )
      {
        // Add more points to the front and end. This is to make sure the zigzag removal doesn't
        // remove important parts of this trajectory.
        Position2d pos;
        Position2d dir;
        ExtendedPose2d first = t->at(0);
        dir = t->at(0).getPosition() - t->at(1).getPosition();
        dir.normalize();
        for( int i = 0; i < 10; i++ )
        {
          // Add a new point before the first:
          pos = first.getPosition() + dir*i*0.005;
          traj.push_back( ExtendedPose2d( pos(0), pos(1), first.getYaw() ) );
        }

        traj.append( *t );

        ExtendedPose2d last = t->at(t->size()-1);
        dir = t->at(t->size()-1).getPosition() - t->at(t->size()-2).getPosition();
        dir.normalize();
        for( int i = 0; i < 10; i++ )
        {
          // Add a new point before the first:
          pos = last.getPosition() + dir*i*0.03;
          traj.push_back( ExtendedPose2d( pos(0), pos(1), last.getYaw() ) );
        }
      } else {
        traj.append( *t );
      }

      lastPatchCenterPose = iterator->getPose();

      // This should be deprecated, but we're too scared to remove it:
      /*if( next )
      {
        bool alreadyVisited = false;
        for( PatchPtrList::iterator it = visited.begin(); it != visited.end(); it++ )
        {
          if( next == (*it) )
          {
            alreadyVisited = true;
            break;
          }
        }
        if( alreadyVisited )
        {
          //LOGGING_INFO( worldLogger, "\tALREADY VISITED!" << t->size() << endl );
          break;
        }
      }*/

      //traj.calculateOrientations();
      lastPose = traj[traj.size()-1];
      if( iterator->getPatchType() == STRAIGHT && iterator->getLane() == LANE_LEFT )
      {
        lastPose.setYaw( lastPose.getYaw() + M_PI );
      }

      if( thisPatchShouldBeFixed )
      {
        iterator->setFixed();
      }

      if( iterator == lastPastPatch )
      {
        thisPatchShouldBeFixed = true;
      } else {
        thisPatchShouldBeFixed = false;
      }

      iterator = next;
      counter++;
    }
  }

  if( mDebugMode )
    writeToFile( traj, "raw" );

  // Ugly workaround because we get poses which lie way off the map.
  // Ignore all poses which are very far away:
  Trajectory2d trajCopy = traj;
  traj.clear();
  for( size_t i = 0; i < trajCopy.size(); i++ )
  {
    if( fabs( trajCopy[i].getX() ) < 1e3 && fabs( trajCopy[i].getY() ) < 1e3 )
    {
      traj.push_back( trajCopy[i] );
    } else {
      LOGGING_WARNING( worldLogger, "Removing trajectory point (too far out)! " <<
          mTrajectoryCounter << endl );
    }
  }

  if( mDebugMode )
    writeToFile( traj, "rawWorkaround" );

  Environment::getInstance()->setRawTrajectory( traj );

  //there can be ZigZag in the trajectory. for example if we are going back at a crossing
  removeZigZag( traj );

  LOGGING_INFO( worldLogger, "Remove ZigZag" << endl );
  Trajectory2d beforeSmooth;
  beforeSmooth = traj;
  //Resampling Intervall. (Traj is stored in this intervall.) we have have 2 * resamplingIntervall constant under the car
  const double resamplingIntervall = 0.05;

  if(useOld && mOldTraj.size() > 0)
  {
    LOGGING_INFO( worldLogger, "we have a old Traj" << endl );
    std::size_t carOnTrajIndex = 0;
    ExtendedPose2d projection;
    double distance;
    //we want to reuse the last points before the car
    Environment::getInstance()->calculateProjection(mOldTraj,Environment::getInstance()->getCarPose(), projection, distance, carOnTrajIndex);

    int lastTrajIndex = std::min(carOnTrajIndex+2, mOldTraj.size()-1);
    Trajectory2d newTrajBack;
    const float maxDistOldTraj = 1;
    //copy old trajectory
    for (int i = 0; i <= lastTrajIndex; i++ ){
      if(Environment::getInstance()->getCar()->calcDistTo(mOldTraj[i]) < maxDistOldTraj){
        newTrajBack.push_back(mOldTraj[i]);
      }
    }

    ExtendedPose2d backEnd = Environment::getInstance()->getCarPose();
    //determine end of old traj
    if(newTrajBack.size() > 0)
    {
      backEnd = newTrajBack.back();
    }
    //give some space for a smooth transiton
    LOGGING_INFO( worldLogger, "add old points" << endl );
    double offset = 0.10;//if you will use this offset you will get surprising results at crossings with 90 degrees
    LOGGING_INFO( worldLogger, "Beginn Resampling. Length of Traj is: " <<traj.length()<<"Traj Point count: "<<traj.size()<< endl );

    beforeSmooth.resample(resamplingIntervall);
    LOGGING_INFO( worldLogger, "Traj is now resampled" << endl );
    bool takeOnePoint = false;
    //add the points of the new trajectory. (of course only the one which are in front of the old points which we have taken)

    //calc projection for the inFrontOfTraj method
    std::size_t projIndex = 0;
    Environment::getInstance()->calculateProjection(beforeSmooth,backEnd, projection, distance, projIndex);

    for (std::size_t i = 0; i < beforeSmooth.size(); i++ ){
      if(isInFrontOnTraj(projection,projIndex, beforeSmooth ,i, offset)){
        newTrajBack.push_back(beforeSmooth[i]);
        takeOnePoint = true;
      }
      else if(takeOnePoint)
      {
        std::cerr<<"remove one point of the traj. but take the point before"<<std::endl;
      }

    }

    traj = newTrajBack;
    //resample the trajectory to avoid bad results in the next interation
    traj.resample(resamplingIntervall);
    //save traj for next iteration
    mOldTraj = traj;


  }
  else if(useOld){
    //we have no old traj so we must take the new one and save the old one
    LOGGING_INFO( worldLogger, "we have no old Traj" << endl );
    traj.resample(resamplingIntervall);
    mOldTraj = traj;
  }
  LOGGING_INFO( worldLogger, "begin smoothing Traj. Size is: " <<traj.size()<< endl );
  //now we smooth the complete Trajectory. The results under the car are hopefully the same. (This is the reason why we use some old points)
  if(traj.size() == 2)
  {
    //interpolate Linear, so we can calculate the curvature
    // no usefull bSpline is possible
    oadrive::core::Interpolator::interpolateLinear(traj,0.5);
  }
  else
  {
    //resample to give the b-Splines more space
    traj.resample(0.30);
    oadrive::core::Interpolator::smoothBSpline(traj);
  }
  //do this things only if we have enough points. otherwise the code will crash
  if(traj.size() >1)
  {
    //now there are a lot of Points due the b Spline. Resample to get smooth curvatures (What would shannon say to this resamplings?)
    traj.calculateOrientations();
    traj.resample(0.15);
    traj.simplify(0.012); //simplify to avoid big curvatures
    traj.calculateCurvature();
  }


  MultiTrajectory multiTraj;
  multiTraj.trajectories.push_back( traj );
  Environment::getInstance()->setTrajectory( multiTraj ); // a traj with 0 points must be set!!!
  LOGGING_INFO( worldLogger, "Traj. is set. Traj. size is: " <<traj.size()<< endl );
  if( traj.size() > 0 )
  {
    return true;
  }
  else
    return false;
}



void TrajectoryFactory::clearOldTraj(){
  mOldTraj.clear();
}


bool TrajectoryFactory::isInFrontOnTraj( ExtendedPose2d& projection,  std::size_t nearest_pose_index,  Trajectory2d traj , std::size_t traj_index, double offset ){

  Trajectory2d newTraj;
  newTraj.append(traj);

  if(traj_index > nearest_pose_index){
    traj_index++;
  }
  else{
    return false;
  }

  newTraj.insert(newTraj.begin()+nearest_pose_index+1, projection);

  double distance = newTraj.lengthBetween(nearest_pose_index+1, traj_index);

  if(distance > offset){
    return true;
  }
  else{
    return false;
  }
}

void TrajectoryFactory::extrapolateFront( Trajectory2d &traj )
{
  if( traj.size() > 0 )
  {
    Trajectory2d copyOfOriginal = traj;
    traj.clear();

    ExtendedPose2d startingPoint = copyOfOriginal[0];
    int pointsToPrepend = 5 - copyOfOriginal.size();

    // If trajectory is too short _prepend_ the first trajectory pose, extrapolated.
    //Eigen::Vector2d dir( cos( startingPoint.getYaw() ), sin( startingPoint.getYaw() ) );
    Position2d dir;
    dir = Position2d( cos( startingPoint.getYaw() ), sin( startingPoint.getYaw() ) );
    dir.normalize();
    for( int i = pointsToPrepend; i > 0; i-- )
    {
      Position2d pos;
      pos = startingPoint.getPosition() - dir*i*0.2;
      traj.push_back( ExtendedPose2d( pos(0), pos(1), 0 ) );
    }

    // At the end, append the original trajectory:
    traj.append( copyOfOriginal );
  }
}




bool TrajectoryFactory::checkTrajectory(Trajectory2d &traj,double maxCurvature)
{
  bool returnValue = true;
  for(unsigned int i = 0; i<traj.size();i++)
  {
    if(traj[i].getCurvature()>maxCurvature||traj[i].getCurvature()<-maxCurvature)
    {
      returnValue = false;
    }

  }
  return returnValue;

}

void TrajectoryFactory::requestUpdate(bool useOld)
{
  if( mTrajectoryMode == PATCH_TRAJECTORY )
  {
    //std::cout << "[TrajectoryFactory] Updating patch trajectory." << std::endl;
    generateFromPatches(useOld);
  } else {
    //std::cout << "[TrajectoryFactory] Fixed trajectory mode. No update needed." << std::endl;
  }
}

Trajectory2d TrajectoryFactory::generateTestTrajectory( std::string trajType )
{
  Trajectory2d traj;
  if( trajType == "rectangle" )
  {
    float sizeX = 3.0;		// circle of 1.5 meters radius.
    float sizeY = 0.7;
    float x = 0;
    float y = 0;
    float stepSize = 0.1;
    for( ; x < sizeX - stepSize; x += stepSize )
    {
      ExtendedPose2d pose( x, -y, 0 );
      traj.push_back( pose );
    }
    for( ; y < sizeY - stepSize; y += stepSize )
    {
      ExtendedPose2d pose( x, -y, 0 );
      traj.push_back( pose );
    }
    for( ; x > 0; x -= stepSize )
    {
      ExtendedPose2d pose( x, -y, 0 );
      traj.push_back( pose );
    }
    for( ; y > 0 + stepSize; y -= stepSize )
    {
      ExtendedPose2d pose( x, -y, 0 );
      traj.push_back( pose );
    }
  }
  else if( trajType == "uTurn" )
  {
    float sizeX = 3.0;		// circle of 1.5 meters radius.
    float sizeY = 0.7;
    float x = 0;
    float y = 0;
    float stepSize = 0.1;
    for( ; x < sizeX - stepSize; x += stepSize )
    {
      ExtendedPose2d pose( x, -y, 0 );
      traj.push_back( pose );
    }
    for( ; y < sizeY - stepSize; y += stepSize )
    {
      ExtendedPose2d pose( x, -y, 0 );
      traj.push_back( pose );
    }
    for( ; x > 0; x -= stepSize )
    {
      ExtendedPose2d pose( x, -y, 0 );
      traj.push_back( pose );
    }
  }
  else if( trajType == "halfCircle" )
  {
    float radius = 2.0;		// circle of 1.5 meters radius.
    float stepSize = M_PI*1.0/36;
    for( float angle = -M_PI/2; angle < M_PI/2 - stepSize; angle += stepSize )
    {
      ExtendedPose2d pose( radius*sin( angle ), -radius + radius*cos( angle ), 0 );
      traj.push_back( pose );
    }

  }
  else if( trajType == "oval" )
  {
    float radius = 1.5;
    float longSection = 1.5;
    float shortSection = 0.9;
    float stepSize = M_PI*1.0/36;
    // long section:
    for( float x = 0; x < longSection; x += 0.2 )
    {
      ExtendedPose2d pose( x, 0, 0 );
      traj.push_back( pose );
    }
    for( float angle = -M_PI/2; angle < 0 - stepSize; angle += stepSize )
    {
      ExtendedPose2d pose( longSection + radius*cos( angle ), radius + radius*sin( angle ), 0 );
      traj.push_back( pose );
    }
    for( float y = 0; y < shortSection; y += 0.2 )
    {
      ExtendedPose2d pose( longSection + radius, radius + y , 0 );
      traj.push_back( pose );
    }
    for( float angle = 0; angle < M_PI/2 - stepSize; angle += stepSize )
    {
      ExtendedPose2d pose( longSection + radius*cos( angle ), radius + shortSection + radius*sin( angle ), 0 );
      traj.push_back( pose );
    }
    for( float x = longSection; x > 0; x -= 0.2 )
    {
      ExtendedPose2d pose( x, shortSection + 2*radius, 0 );
      traj.push_back( pose );
    }
    for( float angle = M_PI/2; angle < M_PI - stepSize; angle += stepSize )
    {
      ExtendedPose2d pose( radius*cos( angle ), radius + shortSection + radius*sin( angle ), 0 );
      traj.push_back( pose );
    }
    for( float y = shortSection; y > 0; y -= 0.2 )
    {
      ExtendedPose2d pose( -radius, radius + y , 0 );
      traj.push_back( pose );
    }
    for( float angle = M_PI; angle < 3*M_PI/2 - stepSize; angle += stepSize )
    {
      ExtendedPose2d pose( radius*cos( angle ), radius + radius*sin( angle ), 0 );
      traj.push_back( pose );
    }

  }
  else
  {	// circle by default
    float radius = 2.0;		// circle of 1.5 meters radius.
    float stepSize = M_PI*2.0/36;
    for( float angle = -stepSize; angle < M_PI*2.0 - stepSize; angle += stepSize )
    {
      ExtendedPose2d pose( radius*sin( angle ), -radius + radius*cos( angle ), 0 );
      traj.push_back( pose );
    }
  }
  return traj;
}





int TrajectoryFactory::removeZigZag( oadrive::core::Trajectory2d &traj ){
  int removed = 0;
  if( traj.size() < 3 ) return removed;
  traj.calculateOrientations();


  ExtendedPose2d fstPose = traj[0];
  for (Trajectory2d::iterator cur=traj.begin()+2; cur != traj.end()-1; )
  {
    ExtendedPose2d fstPose = *(cur-1);
    ExtendedPose2d sndPose = *cur;
    ExtendedPose2d trdPose = *(cur+1);
    double x1 = sndPose.getX() - fstPose.getX();
    double x2 = sndPose.getX() - trdPose.getX();

    double y1 = sndPose.getY() - fstPose.getY();
    double y2 = sndPose.getY() - trdPose.getY();

    double denom1 = std::max( sqrt( x1*x1 + y1*y1 ), 0.000001 );
    double denom2 = std::max( sqrt( x2*x2 + y2*y2 ), 0.000001 );

    x1 = x1 / denom1;
    y1 = y1 / denom1;

    x2 = x2 / denom2;
    y2 = y2 / denom2;

    double angle = acos(x1*x2 + y1*y2);

    //std::cout << "( " << x1 << " , " << y1 << " )  ,  (" << x2 << " , " << y2 << " ) "   << std::endl;
    //std::cout << angle << std::endl;

    if(angle < ZICK_ZACK_ANGLE){

      //raus loeschen
      //traj.erase(traj.begin()+sndInd);
      cur = traj.erase(cur);
      cur = traj.erase(cur);
      if(traj.size()<=3){
        break;
      }
      if(cur != traj.begin()+1){
        cur--;
      }

      removed ++;
      removed ++;
    }
    else{

      cur++;

    }
  }
  return removed;

}
/*bool TrajectoryFactory::canGenerateTrajectory()
  {
  if( mEnvironment->getStreet()->size() > 0 )
  return true;

  return false;
  }*/

Trajectory2d TrajectoryFactory::rotateAndMoveTrajectory( Trajectory2d &traj,
                                                         const ExtendedPose2d &pose )
{
  Trajectory2d rotated;
  for( size_t i = 0; i < traj.size(); i++ )
  {
    // Rotate the trajecotry point acoording to "pose"'s yaw:
    double x = traj[i].getX()*cos( pose.getYaw() ) - traj[i].getY()*sin( pose.getYaw() );
    double y = traj[i].getX()*sin( pose.getYaw() ) + traj[i].getY()*cos( pose.getYaw() );
    // Append position:
    ExtendedPose2d newPose( pose.getX() + x, pose.getY() + y,
                            pose.getYaw() + traj[i].getYaw() );
    rotated.push_back( newPose );
  }
  rotated.isForwardTrajectory() = traj.isForwardTrajectory();
  return rotated;
}

MultiTrajectory TrajectoryFactory::rotateAndMoveTrajectory( MultiTrajectory &multiTraj,
                                                            const ExtendedPose2d &pose )
{
  MultiTrajectory rotatedMultiTrajectory;
  for( unsigned int t = 0; t < multiTraj.trajectories.size(); t++ )
  {
    Trajectory2d rotated, traj;
    traj = multiTraj.trajectories[t];
    for( size_t i = 0; i < traj.size(); i++ )
    {
      // Rotate the trajecotry point acoording to "pose"'s yaw:
      double x = traj[i].getX()*cos( pose.getYaw() ) - traj[i].getY()*sin( pose.getYaw() );
      double y = traj[i].getX()*sin( pose.getYaw() ) + traj[i].getY()*cos( pose.getYaw() );
      // Append position:
      ExtendedPose2d newPose( pose.getX() + x, pose.getY() + y,
                              pose.getYaw() + traj[i].getYaw() );
      rotated.push_back( newPose );
    }
    rotated.isForwardTrajectory() = traj.isForwardTrajectory();
    rotatedMultiTrajectory.trajectories.push_back( rotated );
  }
  return rotatedMultiTrajectory;
}

MultiTrajectory TrajectoryFactory::setNewOrigin( MultiTrajectory &multiTraj,
                                                 const Position2d &origin )
{
  MultiTrajectory movedMultiTrajectory;
  for( unsigned int t = 0; t < multiTraj.trajectories.size(); t++ )
  {
    Trajectory2d moved, traj;
    traj = multiTraj.trajectories[t];
    for( size_t i = 0; i < traj.size(); i++ )
    {
      // Append position:
      ExtendedPose2d newPose( traj[i].getX() - origin(0), traj[i].getY() - origin(1),
                              traj[i].getYaw() );
      moved.push_back( newPose );
    }
    moved.isForwardTrajectory() = traj.isForwardTrajectory();
    movedMultiTrajectory.trajectories.push_back( moved );
  }
  return movedMultiTrajectory;
}

void TrajectoryFactory::startDebugDumping( std::string folder )
{
  mDebugFolder = folder;
  mDebugMode = true;
  mTrajectoryCounter = 0;
}

void TrajectoryFactory::writeToFile( Trajectory2d& traj, std::string name )
{
  std::stringstream path;
  path << mDebugFolder;
  path << name;
  path << mTrajectoryCounter;
  path << ".txt";
  std::ofstream file( path.str().c_str() );
  file << traj; 
}

}   // namespace
}   // namespace



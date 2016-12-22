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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-23
 *
 */
//----------------------------------------------------------------------

#include "Patch.h"
#include <iostream>
#include <limits>
#include <sstream>
#include <iomanip>
#include "TrajectoryDatabase.h"
#include "oadrive_util/Config.h"

using namespace oadrive::core;

#include "worldLogging.h"
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive{
namespace world{

Patch::Patch( PatchType type, const oadrive::core::ExtendedPose2d &pose )
  : EnvObject( pose, PATCH_WIDTHS[type], PATCH_LENGTHS[type] )
  , mOriginalPose( pose )
  , mPatchType( type )
  , mAction(DD_STRAIGHT)
  , mNumAngleRegions( 4 )
  , mIsFixed( false )
{
  //if( mPatchType == CROSS_SECTION )
  //mAction = DD_LEFT;
 
  // Ugly Fix because we get too many
  if( type == PARKING )
  {
    setWidth( PATCH_WIDTHS[type] - 0.10 );
    setHeight( PATCH_LENGTHS[type] - 0.10 );
  }

  setType(PATCH);
  updateTrajectory();

  mPoseVotes.push_back( pose );

  mLane = LANE_RIGHT;
  mSwitchPatch = 0;
  mMagicStreetOffset = oadrive::util::Config::getDouble("Magic","MagicStreetOffset",0.02);
  /*for( float ang = -M_PI*2; ang <= M_PI; ang += 0.01 )
        {
                std::cout << 180.0/M_PI*ang << " " << angle2AngleRegionID( ang ) << std::endl;
        }*/
}

Patch::~Patch(){}



oadrive::core::Trajectory2d* Patch::getTrajectoryFromMC( oadrive::core::ExtendedPose2d& startPose){

  return getTrajectoryFromMC(startPose, mLane);

}

Trajectory2d* Patch::getTrajectoryFromMC( ExtendedPose2d& startPose, enumLane lane ) {
  return getTrajectory(startPose, mAction, lane);
}

Trajectory2d* Patch::getTrajectory( ExtendedPose2d& startPose, drivingDirection dd, enumLane lane ) {


  if(entryPoints.size() < 1) {
    throw( "[Patch] No entry point defined. Call Patch::updateTrajectory() first!" );
  }

  //TODO what to do if the drivingDirection is not available for this kind of patch? Abort? Inform somebody?
  if(mPatchType == STRAIGHT && dd != DD_STRAIGHT) {
    LOGGING_WARNING( worldLogger, "WARNING: STRAIGHT Patch was set to turn left/right!" << endl );
    dd = DD_STRAIGHT;
  }

  double startYaw = startPose.getYaw();

  //check which angle best fits to to startpose
  entryPoint best;
  double best_dist = std::numeric_limits<double>::max();

  for (EntryPointList::iterator it = entryPoints.begin(); it != entryPoints.end(); it++) {
    double distance = fabs((*it).pose.getYaw()  - startYaw);
    while( distance > M_PI )
      distance = fabs( distance - 2.0*M_PI );

    if(distance < best_dist) {
      best_dist = distance;
      best = *it;
    }
  }

  entryDirection goDir = best.dir;


  //         /*LOGGING_WARNING*/( worldLogger, lane << " == " << LANE_LEFT << "  ?  "  << endl );

  if(lane == LANE_LEFT && mPatchType == STRAIGHT){
    //             LOGGING_WARNING( worldLogger, "Change Stuff: " << goDir << "  ,  "  << dd << endl );
    if(goDir == ED_NORTH){
      goDir = ED_SOUTH;
    }
    else{
      goDir = ED_NORTH;
    }
  }

  //         LOGGING_WARNING( worldLogger, "WARNING: " << goDir << "  ,  "  << dd << endl );

  Trajectory2d* traj = &(trajMap[goDir][dd]);

  return traj;
}

Trajectory2d* Patch::getTrajectory( ExtendedPose2d& startPose )
{
  int minTraj = -1;
  double minError = std::numeric_limits<double>::max();
  float TWO_PI = M_PI*2.0;

  // Calculate the distance from the start point to the start points of each trajectory.
  // The closest trajectory will be returned.
  // TODO: Also take direction into account (i.e. compare startPose.getYaw() and the yaw of the
  // first trajectory point).
  for( size_t i = 0; i < mTrajectories.size(); i++ )
  {
    ExtendedPose2d trajPoint = mTrajectories[i](0);
    float dX = trajPoint.getX() - startPose.getX();
    float dY = trajPoint.getY() - startPose.getY();
    float squaredDist = dX*dX + dY*dY;

    float angleDiff = startPose.getYaw() - atan2( dY, dX );
    // Map into -pi .. pi range:
    if( angleDiff > M_PI )
      angleDiff -= TWO_PI;
    if( angleDiff < -M_PI )
      angleDiff += TWO_PI;

    angleDiff = std::abs( angleDiff );

    float error = squaredDist + angleDiff*50;
    /*std::cout << "Error:" << std::endl;
                std::cout << "\tSquared Dist: " << squaredDist << std::endl;
                std::cout << "\tAngle Difference: " << angleDiff << std::endl;
                std::cout << "\tError: " << error << std::endl;*/

    if( error < minError )
    {
      minError = error;
      minTraj = i;
      //std::cout << "new minimum: " << error << " " << i << std::endl;
    }
  }

  // Just in case:
  if( minTraj == -1 )
  {
    throw( "[Patch] No trajectory found. Call Patch::updateTrajectory() first!" );
  }

  return &mTrajectories[minTraj];
}

Trajectory2d rotateAndMoveTrajectory( Trajectory2d &traj,
                                      ExtendedPose2d pose )
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



// TODO:
// Return mutliple trajectories (forward, backward, intersections...)
void Patch::updateTrajectory()
{
  ExtendedPose2d center = getPose();
  double angle = center.getYaw();

  // UnitX points into the direction of the patch, UnitY points towards the left of the patch:
  Position2d unitX, unitY;
  unitX(0) = cos(angle);
  unitX(1) = sin(angle);
  unitY(0) = cos(angle + M_PI*0.5);
  unitY(1) = sin(angle + M_PI*0.5);

  mTrajectories.clear();
  entryPoints.clear();

  if( mPatchType == STRAIGHT )
  {

    // First trajectory is the one on the right lane:
    Trajectory2d t1;
    Position2d lanePoint = center.getPosition() - unitY*(STREET_SIDE_TO_MID_LANE - mMagicStreetOffset);
    Position2d startpoint = center.getPosition() - unitY*(STREET_SIDE_TO_MID_LANE - mMagicStreetOffset) - unitX*PATCH_LENGTHS[STRAIGHT];
    ExtendedPose2d startPose1( startpoint[0], startpoint[1], angle );
    //t1.push_back(startPose1);
    t1.push_back( ExtendedPose2d ( lanePoint[0], lanePoint[1], angle ) );

    trajMap[ED_SOUTH][DD_STRAIGHT] = t1;

    entryPoint ep1;
    ep1.dir = ED_SOUTH;
    ep1.pose = startPose1;
    entryPoints.push_back(ep1);


    // The second trajectory is on the left side (pointing in the opposite direction)
    Trajectory2d t2;
    lanePoint = center.getPosition() + unitY*(STREET_SIDE_TO_MID_LANE - mMagicStreetOffset);

    startpoint = center.getPosition() + unitY*(STREET_SIDE_TO_MID_LANE - mMagicStreetOffset) + unitX*PATCH_LENGTHS[STRAIGHT];
    ExtendedPose2d startPose2( startpoint[0], startpoint[1], angle + M_PI );
    //t2.push_back( startPose2 );
    t2.push_back( ExtendedPose2d( lanePoint[0], lanePoint[1], angle + M_PI ) );

    trajMap[ED_NORTH][DD_STRAIGHT] = t2;

    entryPoint ep2;
    ep2.dir = ED_NORTH;
    ep2.pose = startPose2;
    entryPoints.push_back(ep2);


    mTrajectories.push_back( t1 );
    mTrajectories.push_back( t2 );

  } else if( mPatchType == CROSS_SECTION ) {

    //Direction
    const double lookWest = angle + (M_PI / 2);
    const double lookEast = angle - (M_PI / 2);
    const double lookNorth = angle;
    const double lookSouth = angle + M_PI;

    //Possible Starting Points
    Position2d sSouth = 	center.getPosition()
        - unitX*(PATCH_LENGTHS[CROSS_SECTION]*0.5)
        - unitY*(STREET_SIDE_TO_MID_LANE);
    Position2d sNorth = 	center.getPosition()
        + unitX*(PATCH_LENGTHS[CROSS_SECTION]*0.5)
        + unitY*(STREET_SIDE_TO_MID_LANE);
    Position2d sWest = 	center.getPosition()
        + unitY*(PATCH_LENGTHS[CROSS_SECTION]*0.5)
        - unitX*(STREET_SIDE_TO_MID_LANE);
    Position2d sEast = 	center.getPosition()
        - unitY*(PATCH_LENGTHS[CROSS_SECTION]*0.5)
        + unitX*(STREET_SIDE_TO_MID_LANE);

    //Straight from South
    Position2d startpoint = sSouth;

    entryPoint ep1;
    ep1.dir = ED_SOUTH;
    ep1.pose = ExtendedPose2d( startpoint[0], startpoint[1], lookNorth );
    entryPoints.push_back(ep1);

    Trajectory2d trajCrossStraight = TrajectoryDatabase::getSingleTrajectory( "cross_straight" );
    Trajectory2d trajCrossRight = TrajectoryDatabase::getSingleTrajectory( "cross_right" );
    Trajectory2d trajCrossLeft = TrajectoryDatabase::getSingleTrajectory( "cross_left" );


    trajMap[ED_SOUTH][DD_STRAIGHT] = rotateAndMoveTrajectory( trajCrossStraight, ep1.pose+unitY*mMagicStreetOffset);
    trajMap[ED_SOUTH][DD_RIGHT] = rotateAndMoveTrajectory( trajCrossRight,ep1.pose+unitX*mMagicStreetOffset+unitY*mMagicStreetOffset);
    trajMap[ED_SOUTH][DD_LEFT] = rotateAndMoveTrajectory( trajCrossLeft,ep1.pose-unitX*mMagicStreetOffset+unitY*mMagicStreetOffset);

    // Straight from North
    startpoint = sNorth;

    entryPoint ep2;
    ep2.dir = ED_NORTH;
    ep2.pose = ExtendedPose2d( startpoint[0], startpoint[1], lookSouth );
    entryPoints.push_back(ep2);

    trajMap[ED_NORTH][DD_STRAIGHT] = rotateAndMoveTrajectory( trajCrossStraight, ep2.pose-unitY*mMagicStreetOffset);
    trajMap[ED_NORTH][DD_LEFT] = rotateAndMoveTrajectory(trajCrossLeft, ep2.pose+unitX*mMagicStreetOffset-unitY*mMagicStreetOffset);
    trajMap[ED_NORTH][DD_RIGHT] = rotateAndMoveTrajectory( trajCrossRight, ep2.pose-unitX*mMagicStreetOffset-unitY*mMagicStreetOffset);


    // Straight from West
    startpoint = sWest;

    entryPoint ep3;
    ep3.dir = ED_WEST;
    ep3.pose = ExtendedPose2d( startpoint[0], startpoint[1], lookEast );
    entryPoints.push_back(ep3);

    trajMap[ED_WEST][DD_STRAIGHT] = rotateAndMoveTrajectory( trajCrossStraight, ep3.pose+unitX*mMagicStreetOffset);
    trajMap[ED_WEST][DD_LEFT] = rotateAndMoveTrajectory(trajCrossLeft, ep3.pose+unitX*mMagicStreetOffset+unitY*mMagicStreetOffset);
    trajMap[ED_WEST][DD_RIGHT] = rotateAndMoveTrajectory( trajCrossRight, ep3.pose+unitX*mMagicStreetOffset-unitY*mMagicStreetOffset);

    // Straight from East
    startpoint = sEast;

    entryPoint ep4;
    ep4.dir = ED_EAST;
    ep4.pose = ExtendedPose2d( startpoint[0], startpoint[1], lookWest );
    entryPoints.push_back(ep4);

    trajMap[ED_EAST][DD_STRAIGHT] = rotateAndMoveTrajectory( trajCrossStraight, ep4.pose-unitX*mMagicStreetOffset);
    trajMap[ED_EAST][DD_LEFT] = rotateAndMoveTrajectory(trajCrossLeft, ep4.pose-unitX*mMagicStreetOffset-unitY*mMagicStreetOffset);
    trajMap[ED_EAST][DD_RIGHT] = rotateAndMoveTrajectory( trajCrossRight, ep4.pose-unitX*mMagicStreetOffset+unitY*mMagicStreetOffset);


    //Push all in a list
    mTrajectories.push_back( trajMap[ED_SOUTH][DD_STRAIGHT] );
    mTrajectories.push_back( trajMap[ED_SOUTH][DD_LEFT] );
    mTrajectories.push_back( trajMap[ED_SOUTH][DD_RIGHT] );
    mTrajectories.push_back( trajMap[ED_NORTH][DD_STRAIGHT] );
    mTrajectories.push_back( trajMap[ED_NORTH][DD_LEFT] );
    mTrajectories.push_back( trajMap[ED_NORTH][DD_RIGHT]);
    mTrajectories.push_back( trajMap[ED_WEST][DD_STRAIGHT] );
    mTrajectories.push_back( trajMap[ED_WEST][DD_LEFT] );
    mTrajectories.push_back( trajMap[ED_WEST][DD_RIGHT] );
    mTrajectories.push_back( trajMap[ED_EAST][DD_STRAIGHT] );
    mTrajectories.push_back( trajMap[ED_EAST][DD_LEFT] );
    mTrajectories.push_back( trajMap[ED_EAST][DD_RIGHT] );


  } else if( mPatchType == PARKING ) {

    Trajectory2d t;
    t.push_back( center );
    mTrajectories.push_back( t );

  } else {
    throw( "[Patch] WARNING: Trajectory2d cannot be generated. Unknown patch type!" );
  }
}

//! checks if Patch "other" is on font of the current Patch. (In the same direction as this Patch points)
/*bool Patch::isInFrontOf(PatchPtr other) {
  //TODO: check ob der Patch immer in die Fahrrichtung zeigt! Eventuel kippen falls richtung zu sehr von der Pose des autos abweicht!?

  if(!other)
    return false;

  /////
  //TODO: Doesn change for a Patch! We should calculate this only one time! (At the constructor?)
  double dist_M_B = this->getHeight(); //distance between midpoint and "height" border

  //get outerpoint: point which is on the height plane in yaw direction
  double outX = this->getX() + cos(this->getPose().getYaw()) * dist_M_B;
  double outY = this->getY() + sin(this->getPose().getYaw()) * dist_M_B;
  ////

  *//*
         * Test with  ax + by = c, where the direction (a/b) is vertical to current one
         *//*

  double a = sin(this->getPose().getYaw());
  double b = cos(this->getPose().getYaw());
  double c = a * outX + b * outY;

  *//*
         * ax + t bc > c decides which sides he points lies
         * https://www.c-plusplus.net/forum/266934-full
         *//*

  return a*other->getX() + b* other->getY() > c;

}*/

void Patch::setPose( const ExtendedPose2d &pose )
{
  EnvObject::setPose( pose );
  updateTrajectory();
}

PatchPtr Patch::getChild( unsigned int index )
{
  if( index < mChildren.size() )
  {
    PatchPtrList::iterator it = mChildren.begin();
    std::advance( it, index );
    return *it;
  }
  //if( index < 4 )
  //return mChildren[index];

  return PatchPtr();
}


PatchPtr Patch::getNextChild( ExtendedPose2d &pose )
{
  return getChild( pose, mAction );
}

PatchPtr Patch::getNextChild( float goalAngle )
{
  return getChild( goalAngle, mAction );
}

PatchPtr Patch::getChild(float goalAngle, drivingDirection dir)
{
  //std::cout << "[Patch] getChild() dir: " << dir << std::endl;
  //std::cout << "[Patch] goalAngle: " << goalAngle*180.0/M_PI << std::endl;

  // Calculate the goal angle (do nothing if dir is DD_STRAIGHT ):
  if( dir == DD_RIGHT )
    goalAngle -= M_PI*0.5;
  else if( dir == DD_LEFT )
    goalAngle += M_PI*0.5;

  //std::cout << "[Patch] goalAngle: " << goalAngle*180.0/M_PI << std::endl;

  // Go through the children and find a patch which best fits the angle:
  float minAngle = M_PI*0.5;
  if( mPatchType == CROSS_SECTION )
  {
    minAngle = M_PI*0.3;
  }
  PatchPtr foundChild;

  if(mChildren.size() == 0)
    return foundChild;

  for( PatchPtrList::iterator it = mChildren.begin(); it != mChildren.end(); it++ )
  {
    PatchPtr child = (*it);
    if(!child){
      continue;
    }
    double x = child->getX() - getX();
    double y = child->getY() - getY();
    //Position2d d = (*it)->getPose().getPosition() - getPose().getPosition();
    double childAngle = atan2( y, x );
    //std::cout << "[Patch]\t childAngle: " << childAngle*180.0/M_PI << std::endl;
    double diffAngle =  childAngle - goalAngle;
    //std::cout << "[Patch]\t diffAngle: " << diffAngle*180.0/M_PI << std::endl;
    if( diffAngle >= M_PI )
      diffAngle -= 2*M_PI;
    if( diffAngle <= -M_PI )
      diffAngle += 2*M_PI;
    diffAngle = std::abs( diffAngle );
    //std::cout << "[Patch]\t diffAngle: " << diffAngle*180.0/M_PI << std::endl;

    if( diffAngle < minAngle )
    {
      foundChild = *it;
      minAngle = diffAngle;
    }
  }

  return foundChild;
}

PatchPtr Patch::getChild( ExtendedPose2d &pose, drivingDirection dir )
{
  // Get angle from the given pose to this patch's center:
  Position2d diff = getPose().getPosition() - pose.getPosition();
  float goalAngle = atan2( diff(1), diff(0) );

  return this->getChild(goalAngle,dir);
}

bool Patch::addChild( PatchPtr newChild )
{
  if( newChild->getId() != getId() )
  {
    if( !isChild(newChild) )
    {
      mChildren.push_back( newChild );
      //LOGGING_INFO( worldLogger, "added child " << newChild << endl );
    }
  }

  return true;
}

bool Patch::isChild( PatchPtr other )
{
  /*for( int i = 0; i < 4; i++ )
                if( mChildren[i] == other )
                        return true;*/
  for( PatchPtrList::iterator it = mChildren.begin(); it != mChildren.end(); it++ )
  {
    if( other == (*it) )
      return true;
  }

  return false;
}

// Discretize the angle into mNumAngleRegion regions:
int Patch::angle2AngleRegionID( float angle )
{
  // Independent of my rotation:
  angle = angle - getPose().getYaw();

  // Move by half a region (so that 0 is center of region)
  angle = angle + M_PI*0.25;

  // Wrap into [0, 2*Pi] range (TODO: Check if ExtendedPose2d already does this)
  angle = angle - 2.0*M_PI * floor( angle / (2.0*M_PI) );

  int ID = floor( angle / (2*M_PI) * mNumAngleRegions );
  if( ID == mNumAngleRegions )
    ID = mNumAngleRegions-1;

  return ID;
}

// Convert from the angle region back to an angle:
float Patch::angleRegion2Angle( int regionID )
{
  float angleRegionSize = 2.0*M_PI/mNumAngleRegions;
  return angleRegionSize*regionID + getPose().getYaw();
}

float Patch::getAngleTo( PatchPtr other )
{
  Position2d diff = other->getPose().getPosition() - getPose().getPosition();
  return atan2( diff(1), diff(0) );
}

float Patch::couldBeMyChild( PatchPtr other )
{
  float dist = calcDistTo( other->getPose() );
  // Distance must be large enough (i.e. no overlap).
  // Check the distance dependent on the patch-type:
  if( dist < (getHeight() + other->getHeight())*MINIMUM_PATCH_DIST )
    return -1;

  // Also check against a maximum distance (not patch-type dependent)
  if( dist > MAXIMUM_PATCH_DIST )
    return -1;

  //std::cout << "Could be my child?" << std::endl;
  //std::cout << "\tMy yaw: " << getPose().getYaw()*180.0/M_PI << std::endl;
  //std::cout << "\tOther yaw: " << other->getPose().getYaw()*180.0/M_PI << std::endl;

  float angleToOther = getAngleTo( other );
  //std::cout << "\tangleToOther: " << angleToOther*180.0/M_PI << std::endl;
  int angleRegion = angle2AngleRegionID( angleToOther );
  //std::cout << "\tangleRegion: " << angleRegion << std::endl;
  float angleRegionAngle = angleRegion2Angle( angleRegion );
  //std::cout << "\tangle of angleRegion: " << angleRegionAngle << std::endl;

  // TODO:
  /*if( mPatchType == STRAIGHT && other->getPatchType() == PARKING)
                return true;
        if( mPatchType == PARKING && other->getPatchType() == STRAIGHT )
                return true;*/

  // Angle regions 1 (left)  and 3 (right) not allowed for STRAIGHT patches:
  if( mPatchType == STRAIGHT && (angleRegion == 1 || angleRegion == 3 ) )
    return -1;

  // Calculate offset from "ideal" angle of angle region:
  float angleRegionDiff = std::abs( angleToOther - angleRegionAngle );
  // Get into [-pi, +pi] range:
  if( angleRegionDiff > M_PI )
    angleRegionDiff -= 2.0*M_PI;

  if( std::abs( angleRegionDiff ) > 0.5*M_PI )
    return -1;

  // Check if another patch already occupies the angle region:
  for( PatchPtrList::iterator it = mChildren.begin(); it != mChildren.end(); it++ )
  {
    float angleToOtherChild = getAngleTo( *it );
    int angleRegionChild = angle2AngleRegionID( angleToOtherChild );
    //std::cout << "\t\tOther child: " << angleToOtherChild << " " << angleRegionChild << std::endl;
    if( angleRegionChild == angleRegion )
      return -1;
  }

  return 1.0/( angleRegionDiff + dist + 1e-10 );
}

bool Patch::hasOpenSides()
{
  // Check which sides were already filled with child patches:
  if( mPatchType == STRAIGHT )
  {
    if( mChildren.size() >= 2 )
      return false;
  } else if( mPatchType == CROSS_SECTION )
  {
    if( mChildren.size() >= 4 )
      return false;
  }
  return true;
}

bool Patch::setAction(drivingDirection dir) {

  if(dir != DD_STRAIGHT && mPatchType != CROSS_SECTION) {
    return false;
  }

  mAction = dir;

  return true;
}

drivingDirection Patch::getAction() {
  return mAction;
}

PatchPtrList Patch::getChildren(){
  return mChildren;
}


void Patch::addChildren(PatchPtrList children){
  for(PatchPtrList::iterator it = children.begin();it != children.end(); it++){
    PatchPtr child = *it;
    addChild(child);
  }
}


bool Patch::removeChild(PatchPtr other ){
  if( std::find(mChildren.begin(), mChildren.end(), other) != mChildren.end() )
  {
    mChildren.remove(other);
    return true;
  } else {
    return false;
  }
}


bool Patch::addTrafficSign( TrafficSignPtr newSign )
{
  // First check if the sign already exists in the list:
  if( std::find(mTrafficSigns.begin(), mTrafficSigns.end(), newSign) != mTrafficSigns.end() )
    return false;

  mTrafficSigns.push_back( newSign );


  //check which angle best fits to to startpose
  entryPoint best;
  double best_dist = std::numeric_limits<double>::max();

  for (EntryPointList::iterator it = entryPoints.begin(); it != entryPoints.end(); it++) {
    double distance = newSign->getPose().distance((*it).pose);

    if(distance < best_dist) {
      best_dist = distance;
      best = *it;
    }
  }

  mOrderedTrafficSigns[best.dir] = newSign;
  return true;
}
bool Patch::removeTrafficSign( TrafficSignPtr sign )
{

  for (std::map<entryDirection, TrafficSignPtr>::iterator it=mOrderedTrafficSigns.begin(); it!=mOrderedTrafficSigns.end(); ){
    if(it->second->getId() == sign->getId()){
      mOrderedTrafficSigns.erase(it++);
    }
    else{
      ++it;
    }

  }


  TrafficSignPtrList::iterator it;
  for( it = mTrafficSigns.begin(); it != mTrafficSigns.end(); it++ )
  {
    if( (*it) == sign )
    {
      mTrafficSigns.erase( it );
      return true;
    }
  }
  return false;
}

void Patch::setMonitored(bool monitored){
  mMonitored = monitored;
}
bool Patch::isMonitored(){
  return mMonitored;
}



EventRegionPtr Patch::getObstacleRegion( const ExtendedPose2d &pose , drivingDirection dir )
{
  if(getPatchType() != CROSS_SECTION)
    return EventRegionPtr();

  EventRegionPtr foundEventRegion;

  if( dir == DD_NONE )  // return the center patch
  {
    // Find the Patch closest to the center
    float minDist = std::numeric_limits<float>::max();
    ExtendedPose2d centerPose( 0, 0, 0 );
    for( EventRegionPtrList::iterator it = mEventRegions.begin(); it != mEventRegions.end(); it++ )
    {
      if ((*it)->getEventRegionType() == CROSS_SECTION_OBSTACLES)
      {
        float dist = (*it)->calcDistTo( centerPose );
        if( dist < minDist )
        {
          foundEventRegion = *it;
          minDist = dist;
        }
      }
    }
  } else {
    Position2d diff = getPose().getPosition() - pose.getPosition();
    float goalAngle = atan2( diff(1), diff(0) );

    //std::cout << "[Patch] getChild() dir: " << dir << std::endl;
    //std::cout << "[Patch] goalAngle: " << goalAngle*180.0/M_PI << std::endl;

    LOGGING_INFO( worldLogger, "Attempt to get event region for driving direction: " << dir << endl );
    LOGGING_INFO( worldLogger, "\tgoalAngle: " << goalAngle*180.0/M_PI << endl );

    // Calculate the goal angle (do nothing if dir is DD_STRAIGHT ):
    if( dir == DD_RIGHT )
      goalAngle -= M_PI*0.5;
    else if( dir == DD_LEFT )
      goalAngle += M_PI*0.5;
    /*else if (dir ==  DD_STRAIGHT)
      goalAngle += M_PI;*/

    //LOGGING_INFO( worldLogger, "\tgoalAngle: " << goalAngle*180.0/M_PI << endl );

    // Go through the children and find a patch which best fits the angle:
    float minAngle = M_PI*0.5;

    //LOGGING_INFO( worldLogger, "\tMy event regions: " << mEventRegions.size() << endl );
    for( EventRegionPtrList::iterator it = mEventRegions.begin(); it != mEventRegions.end(); it++ )
    {
      //LOGGING_INFO( worldLogger, "\titeration: " << (*it)->getId() << endl );
      if ((*it)->getEventRegionType() ==  CROSS_SECTION_OBSTACLES)
      {

        //LOGGING_INFO( worldLogger, "\tchecking region" << endl );

        float childAngle = (*it)->getPose().getYaw();
        float diffAngle =  childAngle - goalAngle;
        if( diffAngle >= M_PI )
          diffAngle -= 2*M_PI;
        if( diffAngle <= -M_PI )
          diffAngle += 2*M_PI;
        diffAngle = std::abs( diffAngle );

        //LOGGING_INFO( worldLogger, "\tdiffAngle: " << diffAngle*180.0/M_PI << endl );

        if( diffAngle < minAngle )
        {
          foundEventRegion = *it;
          minAngle = diffAngle;
          LOGGING_INFO( worldLogger, "\t\tmeets criteria." << endl );
        }

      }
    }
  }

  return foundEventRegion;
}


TrafficSignPtr Patch::getCorrespondingTrafficSign(const ExtendedPose2d &pose ){

  Position2d diff = getPose().getPosition() - pose.getPosition();
  float goalAngle = atan2( diff(1), diff(0) ) + M_PI_4 ;

  TrafficSignPtr sign;
  float minAngle = M_PI_2;

  TrafficSignPtrList::iterator it;
  for( it = mTrafficSigns.begin(); it != mTrafficSigns.end(); it++ )
  {

    Position2d diff2 = (*it)->getPose().getPosition() - getPose().getPosition();
    float goalAngle2 = atan2( diff2(1), diff2(0) );

    float diffAngle = goalAngle - goalAngle2;
    if( diffAngle >= M_PI )
      diffAngle -= 2.0*M_PI;
    if( diffAngle <= -M_PI )
      diffAngle += 2.0*M_PI;
    diffAngle = std::abs( diffAngle );

    if( diffAngle < minAngle){
      minAngle = diffAngle;
      sign = (*it);
    }

  }
  return sign;
}

bool sortByAngle( const ExtendedPose2d &p1, const ExtendedPose2d &p2 )
{
  return p1.getYaw() < p2.getYaw();
}

bool Patch::tryMerge( PatchPtr otherPatch )
{
  if( mIsFixed )
    return false;

  if( mPatchType != otherPatch->getPatchType() )
    return false;

  // If this is a parking patch, "merge", by sorting all possible poses by angle and taking
  // the median pose.
  if( mPatchType == PARKING )
  {
    ExtendedPose2d newVote = otherPatch->getPose();
    float yaw = newVote.getYaw();
    // Map into 0 .. pi range:
    while( yaw >= M_PI )
      yaw -= M_PI;
    while( yaw < 0 )
      yaw += M_PI;

    newVote.setYaw( yaw );

    mPoseVotes.push_back( newVote );

    std::sort( mPoseVotes.begin(), mPoseVotes.end(), sortByAngle );

    unsigned int index = floor( mPoseVotes.size()/2.0 );
    setPose( mPoseVotes[index] );

  } else {
    // update the reference patch by also taking into account the new patch:
    Position2d p1 = getPose().getPosition();
    Position2d p2 = otherPatch->getPose().getPosition();
    Position2d unit1, unit2, yawVec;
    float angle1 = getPose().getYaw();
    float angle2 = otherPatch->getPose().getYaw();

    // If the difference is larger than +-90°, rotate the new patch
    // by 180°.
    float diff = angle2 - angle1;
    if( diff > M_PI )
      angle2 -= 2.0*M_PI;
    else if( diff < -M_PI )
      angle2 += 2.0*M_PI;

    diff = angle2 - angle1;
    if( diff > M_PI_2 )
      angle2 -= M_PI;
    else if( diff < -M_PI_2 )
      angle2 += M_PI;
    diff = angle2 - angle1;

    if( otherPatch->getPatchType() == CROSS_SECTION )
    {
      if( diff > M_PI_4 )
        angle2 -= M_PI_2;
      else if( diff < -M_PI_4 )
        angle2 += M_PI_2;
    }

    unit1(0) = cos( angle1 );
    unit1(1) = sin( angle1 );
    unit2(0) = cos( angle2 );
    unit2(1) = sin( angle2 );
    yawVec = unit1*getProbability() + unit2*otherPatch->getProbability()*2.0;
    //yawVec = unit1 + unit2;

    float weight = otherPatch->getProbability()/
      (otherPatch->getProbability()+getProbability());

    Position2d p = p1 + (p2-p1)*(weight);

    setPose( ExtendedPose2d( p(0), p(1), atan2( yawVec(1), yawVec(0) ) ) );
    setProbability( getProbability() + otherPatch->getProbability() );
  }
  return false;
}




bool Patch::isSwitchPatch(){
  return mSwitchPatch > 0;
}
int Patch::getSwitchType(){
  return mSwitchPatch;
}
void Patch::setSwitchType(int swType){
  mSwitchPatch = swType;
}

enumLane Patch::getLane(){
  return mLane;
}

void Patch::setLane(enumLane lane){
  mLane = lane;
}

std::string Patch::toJson() {

  std::stringstream sstr;
  sstr << std::setprecision( 2 );
  sstr << std::fixed;
  sstr << "{" <<
    "\"type\": \"patch\"," <<
    "\"id\": " << this->getId() << "," <<
    "\"patchType\": " << this->getPatchType() << "," <<
    "\"x\": " << this->getX() << "," <<
    "\"y\": " << this->getY() << "," <<
    "\"yaw\": " << this->getYaw() <<
    "} ";
  return sstr.str();

}

}	// namespace
}	// namespace


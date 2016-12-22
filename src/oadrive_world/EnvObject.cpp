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
 * \date    2015-12-01
 *
 */
//----------------------------------------------------------------------

#include "EnvObject.h"
#include <oadrive_world/worldLogging.h>
#include <exception>

#include "EventRegion.h"

using icl_core::logging::endl;
using namespace oadrive::core;

namespace oadrive{
namespace world{

class EventRegion;
typedef boost::shared_ptr<EventRegion> EventRegionPtr;

const double TWO_PI = 2.0 * M_PI;

unsigned int EnvObject::IDCounter = 0;

EnvObject::EnvObject(const oadrive::core::ExtendedPose2d &pose, double width, double height) {
  this->mPose = pose;
  this->mProbability = 1;
  this->mLikelyhood = 0;
  this->mType = 0;
  this->mHeight = height;
  this->mWidth = width;
  this->mTime = std::time(0);
  this->mId = IDCounter++;
  updateCorners();
}

EnvObject::~EnvObject() {
  // TODO Auto-generated destructor stub
}


oadrive::core::ExtendedPose2d EnvObject::getPose() const {
  return mPose;
}

void EnvObject::setPose(const oadrive::core::ExtendedPose2d &pose) {
  this->mPose = pose;
  this->updateCorners();
  this->updateTime();
  if( mEventRegions.size() > 0 )
  {
    for( EventRegionPtrList::iterator it = mEventRegions.begin();
         it != mEventRegions.end(); it++ )
    {
      (*it)->setPose( toWorldCoords( (*it)->getLocalPose() ) );
    }
  }
}

double EnvObject::getProbability() const {
  return mProbability;
}
void EnvObject::setProbability(double probability) {
  this->mProbability = std::min( probability, 1.0 );
}
double EnvObject::getLikelyhood() const {
  return mLikelyhood;
}
void EnvObject::setLikelyhood(double likelyhood) {
  this->mLikelyhood = likelyhood;
}
int EnvObject::getType() const {
  return mType;
}

void EnvObject::setType(int type) {
  this->mType = type;
}

double EnvObject::getX() {
  return this->mPose.getX();
}

void EnvObject::setX(double x) {
  this->mPose.setX(x);
  this->updateCorners();
  this->updateTime();
}

double EnvObject::getY() {
  return this->mPose.getY();
}

void EnvObject::setY(double y) {
  this->mPose.setY(y);
  this->updateCorners();
  this->updateTime();
}
double EnvObject::getYaw() {
  return mPose.getYaw();
}

void EnvObject::setYaw(double yaw) {
  mPose.setYaw(yaw);
  this->updateCorners();
  this->updateTime();
}


double EnvObject::getHeight() const {
  return mHeight;
}

void EnvObject::setHeight(double height) {
  this->mHeight = height;
  this->updateTime();
}

double EnvObject::getWidth() const {
  return mWidth;
}

void EnvObject::setWidth(double width) {
  this->mWidth = width;
  this->updateTime();
}

double EnvObject::getTime() const {
  return mTime;
}

void EnvObject::setTime(double time) {
  this->mTime = time;
}

unsigned int EnvObject::getId() const {
  return mId;
}

//TODO: alle calcDist durch eine mit aufruf bei den anderen ersetzen
double EnvObject::calcDistTo(EnvObjectPtr other) {
  return calcDistTo(other->getPose());
}


double EnvObject::calcDistTo( const ExtendedPose2d &pose ) {
  return calcDistTo(pose.getPosition());
}

double EnvObject::calcDistTo( const Position2d &pos ) {
  return std::sqrt( (this->getX() - pos[0] )  * (this->getX() -  pos[0]) + (this->getY() -  pos[1])  * (this->getY() -  pos[1]) );
}


void EnvObject::calcTempDist(const ExtendedPose2d &pose){

  this->updateCorners();
  Position2d pos = pose.getPosition();
  double distA = std::sqrt( (this->getCornerA()[0] - pos[0] )  * (this->getCornerA()[0] - pos[0] ) + (this->getCornerA()[1] - pos[1] )  * (this->getCornerA()[1] - pos[1] ) );
  double distB = std::sqrt( (this->getCornerB()[0] - pos[0] )  * (this->getCornerB()[0] - pos[0] ) + (this->getCornerB()[1] - pos[1] )  * (this->getCornerB()[1] - pos[1] ) );
  double distC = std::sqrt( (this->getCornerC()[0] - pos[0] )  * (this->getCornerC()[0] - pos[0] ) + (this->getCornerC()[1] - pos[1] )  * (this->getCornerC()[1] - pos[1] ) );
  double distD = std::sqrt( (this->getCornerD()[0] - pos[0] )  * (this->getCornerD()[0] - pos[0] ) + (this->getCornerD()[1] - pos[1] )  * (this->getCornerD()[1] - pos[1] ) );
  double distM = calcDistTo(pos);

  this->mTempDist = std::min(distA,std::min(distB,std::min(distC,std::min(distD,distM))));

}

double EnvObject::getTempDist(){
  return mTempDist;
}


bool EnvObject::contains(double x, double y){

  if( (x >= this->getX() && x <= (this->getX()+this->getWidth()) ) && (y >= this->getY() && y <= (this->getY()+this->getHeight()) ) ){
    return true;
  }
  else{
    return false;
  }
}

//calculates the angle offset from this yaw to the direction to a given Object
// This angle will be in the range [0,pi]
double EnvObject::calcAngleOffset(EnvObject *otherObject){

  return this->calcAngleOffset(otherObject->mPose);
}

double EnvObject::calcAngleOffset(const ExtendedPose2d &pose){

  double x1 = cos(this->mPose.getYaw());
  double y1 = sin(this->mPose.getYaw());

  double x2 = pose.getX() - this->mPose.getX();
  double y2 = pose.getY() - this->mPose.getY();

  // Ugly:
  double denom = std::max( sqrt( x2*x2 + y2*y2 ), 0.000001 );

  x2 = x2 / denom;
  y2 = y2 / denom;

  double angle = acos(x1*x2 + y1*y2);

  return angle;
}


void EnvObject::updateTime(){
  this->setTime(std::time(0));
}


bool EnvObject::isPointInside( const ExtendedPose2d &M )
{

  // Implements test:
  // (0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
  // Taken from Stack-Overflow:
  // http://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
  // ( 22. December 2015 )
  float AM_x = M.getX() - mCornerA(0);
  float AM_y = M.getY() - mCornerA(1);

  float AM_AB = AM_x*mAB(0) + AM_y*mAB(1);
  float AM_AD = AM_x*mAD(0) + AM_y*mAD(1);

  if( 0 < AM_AB && AM_AB < mAB_AB
      && 0 < AM_AD && AM_AD < mAD_AD )
  {
    return true;
  }
  return false;
}

/*
bool EnvObject::isPointInside( Eigen::Vector2d M )
{
        // Implements test:
        // (0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        // Taken from Stack-Overflow:
        // http://math.stackexchange.com/questions/190111/how-to-check-if-a-point-is-inside-a-rectangle
        // ( 22. December 2015 )
        float AM_x = M(0) - mCornerA(0);
        float AM_y = M(1) - mCornerA(1);

        float AM_AB = AM_x*mAB(0) + AM_y*mAB(1);
        float AM_AD = AM_x*mAD(0) + AM_y*mAD(1);

        if( 0 < AM_AB && AM_AB < mAB_AB
                        && 0 < AM_AD && AM_AD < mAD_AD )
        {
                return true;
        }
        return false;
}
*/
void EnvObject::updateCorners()
{
  try {


    float angle = getPose().getYaw();
    // UnitX points into the direction of the patch, UnitY points towards the left of the patch:

    //LOGGING_INFO(worldLogger,"updateCorners past yaw"  << endl);

    oadrive::core::Position2d unitX, unitY;
    unitX(0) = cos(angle);
    unitX(1) = sin(angle);
    unitY(0) = cos(angle + M_PI*0.5);
    unitY(1) = sin(angle + M_PI*0.5);

    //LOGGING_INFO(worldLogger,"updateCorners past position2d"  << endl);

    mCornerA = -0.5*getHeight()*unitX + 0.5*getWidth()*unitY + getPose().getPosition();
    mCornerB = 0.5*getHeight()*unitX + 0.5*getWidth()*unitY + getPose().getPosition();
    mCornerC = 0.5*getHeight()*unitX - 0.5*getWidth()*unitY + getPose().getPosition();
    mCornerD = -0.5*getHeight()*unitX - 0.5*getWidth()*unitY + getPose().getPosition();

    //LOGGING_INFO(worldLogger,"updateCorners past init Corners"  << endl);

    mAB = mCornerB - mCornerA;
    mAD = mCornerD - mCornerA;

    mAB_AB = mAB(0)*mAB(0) + mAB(1)*mAB(1);
    mAD_AD = mAD(0)*mAD(0) + mAD(1)*mAD(1);

  } catch (std::exception &e) {
    LOGGING_INFO(worldLogger,"exception in update corners with:" << e.what() << endl);
  }
}

unsigned int EnvObject::checkIntersections( const Position2d &start, const Position2d &end )
{
  // TODO: Don't recalculate corners for every check?
  float angle = getPose().getYaw();
  // UnitX points into the direction of the patch, UnitY points towards the left of the patch:
  Position2d unitX, unitY;
  unitX(0) = cos(angle);
  unitX(1) = sin(angle);
  unitY(0) = cos(angle + M_PI*0.5);
  unitY(1) = sin(angle + M_PI*0.5);

  // Define four corner points:
  Position2d mCornerA = -0.5*getHeight()*unitX + 0.5*getWidth()*unitY + getPose().getPosition();
  Position2d mCornerB = 0.5*getHeight()*unitX + 0.5*getWidth()*unitY + getPose().getPosition();
  //Position2d mCornerC = 0.5*getHeight()*unitX - 0.5*getWidth()*unitY + getPose().getPosition();
  Position2d mCornerD = -0.5*getHeight()*unitX - 0.5*getWidth()*unitY + getPose().getPosition();

  Position2d d1 = mCornerB - mCornerA;
  Position2d d2 = end - start;

  unsigned int intersections = 0;
  if( lineSegmentIntersection( mCornerA, start, d1, d2 ) )
    intersections ++;
  if( lineSegmentIntersection( mCornerD, start, d1, d2 ) )
    intersections ++;

  d1 = mCornerD - mCornerA;

  if( lineSegmentIntersection( mCornerA, start, d1, d2 ) )
    intersections ++;
  if( lineSegmentIntersection( mCornerB, start, d1, d2 ) )
    intersections ++;

  return intersections;
}

double EnvObject::crossProduct( const Position2d &p1, const Position2d &p2 )
{
  return p1[0]*p2[1] - p1[1]*p2[0];
}

bool EnvObject::lineSegmentIntersection( const Position2d &p, const Position2d &q,
                                         const Position2d &r, const Position2d &s )
{
  double rxs = crossProduct( r, s );

  if( rxs == 0 )
    return false;

  //double pxq = crossProduct( p, q );
  double p_qxr = crossProduct( q-p, r );
  double p_qxs = crossProduct( q-p, s );

  //u = (q − p) × r / (r × s);
  //t = (q − p) × s / (r × s);

  double u = p_qxr/rxs;
  double t = p_qxs/rxs;

  if( u >= 0 && u <= 1 && t >= 0 && t <= 1 )
  {
    // Intersection point is:
    // p + t*r;
    return true;
  }
  return false;
}        

ExtendedPose2d EnvObject::toWorldCoords( const ExtendedPose2d &localPos )
{
  double ang = mPose.getYaw();
  double x = mPose.getX() + localPos.getX()*cos( ang ) - localPos.getY()*sin( ang );
  double y = mPose.getY() + localPos.getX()*sin( ang ) + localPos.getY()*cos( ang );
  return ExtendedPose2d( x, y, localPos.getYaw() + ang );
}

oadrive::core::Position2d EnvObject::getCornerA(){
  return this->mCornerA;
}       
oadrive::core::Position2d EnvObject::getCornerB(){
  return this->mCornerB;
}       
oadrive::core::Position2d EnvObject::getCornerC(){
  return this->mCornerC;
}       
oadrive::core::Position2d EnvObject::getCornerD(){
  return this->mCornerD;
}

bool EnvObject::checkOverlap(EnvObjectPtr other){

  if(!other)
    return false;

  bool overlap = false;
  this->updateCorners();
  other->updateCorners();

  ExtendedPose2d thisCornerA(this->getCornerA()(0),this->getCornerA()(1),0);
  ExtendedPose2d thisCornerB(this->getCornerB()(0),this->getCornerB()(1),0);
  ExtendedPose2d thisCornerC(this->getCornerC()(0),this->getCornerC()(1),0);
  ExtendedPose2d thisCornerD(this->getCornerD()(0),this->getCornerD()(1),0);

  ExtendedPose2d otherCornerA(other->getCornerA()(0),other->getCornerA()(1),0);
  ExtendedPose2d otherCornerB(other->getCornerB()(0),other->getCornerB()(1),0);
  ExtendedPose2d otherCornerC(other->getCornerC()(0),other->getCornerC()(1),0);
  ExtendedPose2d otherCornerD(other->getCornerD()(0),other->getCornerD()(1),0);


  if(this->isPointInside(otherCornerA) || this->isPointInside(otherCornerB) || this->isPointInside(otherCornerC) || this->isPointInside(otherCornerD)){
    overlap = true;
  }
  if(other->isPointInside(thisCornerA) || other->isPointInside(thisCornerB) || other->isPointInside(thisCornerC) || other->isPointInside(thisCornerD)){
    overlap = true;
  }

  checkIntersections((other->getCornerA()), (other->getCornerB()));
  /*
    if(this->checkIntersections(&(other->getCornerA()), &(other->getCornerB()))>0
        || this->checkIntersections(other->getCornerB(), other->getCornerC())>0
        || this->checkIntersections(other->getCornerC(), other->getCornerD())>0
         || this->checkIntersections(other->getCornerD(), other->getCornerA())>0  ){
        overlap = true;
    }
    */


  return overlap;
}



int EnvObject::getTimeSinceLastUpdate(){
  return (std::time(0)-mTime);
}

EventRegionPtr EnvObject::getEventRegion()
{
  if( mEventRegions.size() > 0 )
    return mEventRegions.front();
  else
    return EventRegionPtr();
}

void EnvObject::addEventRegion( EventRegionPtr evRegion )
{
  mEventRegions.push_back( evRegion );
  evRegion->setPose( toWorldCoords( evRegion->getLocalPose() ) );
}

void EnvObject::clearEventRegions()
{
  mEventRegions.clear();
}

void EnvObject::removeEventRegion(EventRegionPtr evRegion)
{
  mEventRegions.remove( evRegion );
}



}	// namespace
}	// namespace

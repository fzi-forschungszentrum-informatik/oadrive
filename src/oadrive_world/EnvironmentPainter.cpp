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
 * \date    2016-02-08
 *
 */
//----------------------------------------------------------------------

#include "EnvironmentPainter.h"
#include "Environment.h"
#include <oadrive_util/Config.h>
#include <sstream>
#include <boost/lexical_cast.hpp>

using namespace boost::posix_time;

namespace oadrive{
namespace world{

using namespace oadrive::core;

EnvironmentPainter::EnvironmentPainter()
{
  mDrawEventRegions = Config::getBool( "EnvironmentPainter", "drawEventRegions", true );
  //mDrawHistory = Config::getBool( "EnvironmentPainter", "drawHistory", true );
  mDrawObstacleDistances = Config::getBool( "EnvironmentPainter",
      "drawObstacleDistances", false );

  mCameraPosX = Config::getDouble( "BirdviewCal", "mCameraPosX", -0.055 );
  mCameraPosY = Config::getDouble( "BirdviewCal", "mCameraPosY", 0.295 );
  mCameraPosZ = Config::getDouble( "BirdviewCal", "mCameraPosZ", 0.225 );
  mCameraPitch = Config::getDouble( "BirdviewCal", "mCameraPitch", -0.05 );

}

void EnvironmentPainter::drawPatches( Environment* env, cv::Mat &img, double x, double y,
                                      double radius, float pixelsPerMeter )
{
  cv::Scalar col = cv::Scalar( 0,0,64 );
  /*for ( PatchPtrList::iterator it = env->mUnsortedPatches.begin(); it != env->mUnsortedPatches.end(); it++)
  {
    PatchPtr patch = *it;
    double realX = -x + patch->getPose().getX() + radius;
    double realY = -y + patch->getPose().getY() + radius;
    cv::Point2f center( realX*pixelsPerMeter, realY*pixelsPerMeter );
    cv::RotatedRect rRect = cv::RotatedRect( center,
                                             cv::Size2f( patch->getHeight()*pixelsPerMeter, patch->getWidth()*pixelsPerMeter ),
                                             patch->getPose().getYaw()*180.0/M_PI );

    cv::Point2f vertices[4];
    rRect.points(vertices);
    cv::Scalar pCol = col;//+col*patch->getProbability()*50;
    pCol *= 0.75;
    for (int i = 0; i < 4; i++)
      cv::line(img, vertices[i], vertices[(i+1)%4], pCol, 1 );

    // Draw direction:
    cv::Point2f direction( 10*cos(patch->getPose().getYaw()), 10*sin(patch->getPose().getYaw()) );
    cv::line( img, center, center + direction, cv::Scalar(255,255,255) );
    cv::circle( img, center, 2, cv::Scalar(255,255,255) );
  }*/

  for ( PatchPtrList::iterator it = env->mStreet.begin(); it != env->mStreet.end(); it++ )
  {
    PatchPtr patch = *it;
    double realX = -x + patch->getPose().getX() + radius;
    double realY = -y + patch->getPose().getY() + radius;
    cv::Point2f center( realX*pixelsPerMeter, realY*pixelsPerMeter );
    cv::RotatedRect rRect = cv::RotatedRect( center,
                                             cv::Size2f( patch->getHeight()*pixelsPerMeter, patch->getWidth()*pixelsPerMeter ),
                                             patch->getPose().getYaw()*180.0/M_PI );

    if( patch->hasOpenSides() )
      col = cv::Scalar( 255,128,0 );
    else
      col = cv::Scalar( 64,128,255 );

    cv::Point2f vertices[4];
    rRect.points(vertices);
    //cv::Scalar pCol = col;//+col*patch->getProbability()*20;
    //pCol *= 0.3;
    int width = 1;
    if( patch->isFixed() )
      width = 2;
    for (int i = 0; i < 4; i++)
      cv::line(img, vertices[i], vertices[(i+1)%4], col, width );

    // Draw connection lines to my children:
    PatchPtrList children = patch->getChildren();
    for( PatchPtrList::iterator child = children.begin(); child != children.end(); child++ )
    {
      double realX_other = -x + (*child)->getPose().getX() + radius;
      double realY_other = -y + (*child)->getPose().getY() + radius;
      cv::Point2f center_other( realX_other*pixelsPerMeter, realY_other*pixelsPerMeter);
      cv::Scalar col( 0,0,100 );
      cv::line(img, center, center_other, col, 1 );
    }

    // Draw patch pose direction:
    cv::Point2f direction( 10*cos(patch->getPose().getYaw()), 10*sin(patch->getPose().getYaw()) );
    cv::line( img, center, center + direction, cv::Scalar(255,255,255) );
    cv::circle( img, center, 2, cv::Scalar(255,255,255) );

  }

  for ( PatchPtrList::iterator it = env->mParkingLots.begin(); it != env->mParkingLots.end(); it++ )
  {
    PatchPtr patch = *it;
    double realX = -x + patch->getPose().getX() + radius;
    double realY = -y + patch->getPose().getY() + radius;
    cv::Point2f center( realX*pixelsPerMeter, realY*pixelsPerMeter );
    cv::RotatedRect rRect = cv::RotatedRect( center,
                                             cv::Size2f( patch->getHeight()*pixelsPerMeter, patch->getWidth()*pixelsPerMeter ),
                                             patch->getPose().getYaw()*180.0/M_PI );

    cv::Scalar col = cv::Scalar( 64,128,255 );

    cv::Point2f vertices[4];
    rRect.points(vertices);
    //cv::Scalar pCol = col;//+col*patch->getProbability()*20;
    //pCol *= 0.3;
    for (int i = 0; i < 4; i++)
      cv::line(img, vertices[i], vertices[(i+1)%4], col, 1 );

    // Draw patch pose direction:
    cv::Point2f direction( 10*cos(patch->getPose().getYaw()), 10*sin(patch->getPose().getYaw()) );
    cv::line( img, center, center + direction, cv::Scalar(255,255,255) );
    cv::circle( img, center, 2, cv::Scalar(255,255,255) );

  }

}
void EnvironmentPainter::drawObstacles( Environment* env, cv::Mat &img, double x, double y,
                                        double radius, float pixelsPerMeter )
{
  ObstaclePtrList::iterator it;
  ExtendedPose2d car(x,y,0);
  for( it = env->mObstacles.begin(); it != env->mObstacles.end(); )
  {
    if((car.distance((*it)->getPose())<radius))//&&((*it)->getProbability())>0.7)
    {
      double width = (*it)->getWidth();
      double height = (*it)->getHeight();
      double realX = -x + (*it)->getPose().getX() + radius;
      double realY = -y + (*it)->getPose().getY() + radius;

      if( mDrawObstacleDistances )
      {
        cv::Point2f obstaclePos( realX*pixelsPerMeter,
            realY*pixelsPerMeter);
        double carX = -x + env->mCar->getPose().getX() + radius;
        double carY = -y + env->mCar->getPose().getY() + radius;
        cv::Point2f carPos( carX*pixelsPerMeter,
            carY*pixelsPerMeter);
        cv::line( img, obstaclePos, carPos, cv::Scalar( 128,128,128 ) );

        std::stringstream sstr;
        sstr << env->mCar->calcDistTo( *it ) << "m";
        cv::putText(img, sstr.str(), obstaclePos + cv::Point2f( 15, 0 ),
            cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.6, cv::Scalar::all(255), 1 );
      }

      width  = std::max(width,0.05);
      height = std::max(height,0.05);
      cv::Point2f position1( (realX-width/2)*pixelsPerMeter, (realY-height/2)*pixelsPerMeter);
      cv::Point2f position2( (realX+width/2)*pixelsPerMeter, (realY+height/2)*pixelsPerMeter);

      int thickness = 1;
      if( (*it)->isStatic() )
      {
        thickness = CV_FILLED;
      }
      if(((*it)->getSawUS()>0)&&!((*it)->getSawDepth()>0))
      {
        cv::rectangle(img, position1, position2, cv::Scalar(0,140,255), thickness );
      }
      else if(!((*it)->getSawUS()>0)&&((*it)->getSawDepth()>0))
      {
        cv::rectangle(img, position1, position2, cv::Scalar(0,255,140), thickness );
      }
      else
      {
        cv::rectangle(img, position1, position2, cv::Scalar(140,255,140), thickness );
      }

      cv::Point2f position3( (realX-(width)/2)*pixelsPerMeter-2, (realY-height/2)*pixelsPerMeter-2);
      cv::Point2f position4( (realX+width/2)*pixelsPerMeter+2, (realY+height/2)*pixelsPerMeter+2);
      if( (*it)->getProbability() > EXISTENCE_THRES )
      {
        cv::rectangle(img, position3, position4, cv::Scalar(0,0,255) );
      } else {
        cv::rectangle(img, position3, position4, cv::Scalar(100,100,100) );
      }

      /*if((*it)->getIsRelevantAccDebugOnly())
      {
        cv::Point2f position1( (realX-(width)/2)*pixelsPerMeter-2, (realY-height/2)*pixelsPerMeter-2);
        cv::Point2f position2( (realX+width/2)*pixelsPerMeter+2, (realY+height/2)*pixelsPerMeter+2);
        cv::rectangle(img, position1, position2, cv::Scalar(0,0,255) );
      }*/

    }
    it++;
  }
}

void EnvironmentPainter::drawEventRegions( Environment* env, cv::Mat &img, double x, double y,
                                           double radius, float pixelsPerMeter )
{
  cv::Scalar col;
  for ( EventRegionPtrList::iterator it = env->mEventRegions.begin();
        it != env->mEventRegions.end(); it++)
  {
    EventRegionPtr region = *it;

    if( region->getEventRegionType() == OBSTACLE_REGION )
    {
      continue;
      /*ObstaclePtr obstacle = boost::dynamic_pointer_cast<Obstacle>( region->getObjectOfInterest() );
      if( obstacle->getProbability() <= 0.7 )
        continue;*/
    }
    /*if( !region->getIsCarInside() )
                        continue;*/

    double realX = -x + region->getPose().getX() + radius;
    double realY = -y + region->getPose().getY() + radius;
    cv::Point2f center( realX*pixelsPerMeter, realY*pixelsPerMeter );
    cv::RotatedRect rRect = cv::RotatedRect( center,
                                             cv::Size2f( region->getHeight()*pixelsPerMeter, region->getWidth()*pixelsPerMeter ),
                                             region->getPose().getYaw()*180.0/M_PI );

    if( region->getIsCarInside() )
      col = cv::Scalar( 32, 32, 64 );
    else
      col = cv::Scalar( 32, 64, 32 );

    cv::Point2f vertices[4];
    rRect.points(vertices);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertices[i], vertices[(i+1)%4], col, 1 );
  }
}

void EnvironmentPainter::drawTrafficSign( Environment* env, cv::Mat &img, double x, double y,
                                          double radius, float pixelsPerMeter )
{
  TrafficSignPtrList::iterator it;
  ExtendedPose2d car(x,y,0);
  for( it = env->mTrafficSigns.begin(); it != env->mTrafficSigns.end(); )
  {
    if(car.distance((*it)->getPose())<radius)
    {
      double width = (*it)->getWidth();
      double height = (*it)->getHeight();
      double realX = -x + (*it)->getPose().getX() + radius;
      double realY = -y + (*it)->getPose().getY() + radius;
      width  = std::max(width,0.05);
      height = std::max(height,0.05);

      cv::Scalar color(255, 159, 35);

      if( (*it)->getConnectedPatch() )
      {
        double realXPatch = -x + (*it)->getConnectedPatch()->getPose().getX() + radius;
        double realYPatch = -y + (*it)->getConnectedPatch()->getPose().getY() + radius;
        cv::Point2f center( realX*pixelsPerMeter, realY*pixelsPerMeter);
        cv::Point2f patchCenter( realXPatch*pixelsPerMeter, realYPatch*pixelsPerMeter );
        cv::line( img, center, patchCenter, color );
      }

      cv::Point2f direction( 10*cos((*it)->getPose().getYaw()), 10*sin((*it)->getPose().getYaw()) );
      cv::Point2f position( realX*pixelsPerMeter, realY*pixelsPerMeter);
      if((*it)->getProbability() <= TRAFFIC_SIGN_MINIMUM_CONNECTION_PROBABLITY) {
        color = cv::Scalar(143, 81, 0);
      }

      cv::line( img, position, position + direction, color, 1 );
      cv::circle( img, position, 2, color );
    }
    it++;
  }

}

void EnvironmentPainter::drawHistory( Environment* env, cv::Mat &img, double x, double y,
                                      double radius, float pixelsPerMeter )
{
  cv::Scalar col = cv::Scalar( 0,255,0 );
  for (PatchPtrList::iterator it = env->mPastCarPatches.begin();
       it != env->mPastCarPatches.end(); it++)
  {
    PatchPtr patch = *it;
    double realX = -x + patch->getPose().getX() + radius;
    double realY = -y + patch->getPose().getY() + radius;
    cv::Point2f center( realX*pixelsPerMeter, realY*pixelsPerMeter );
    cv::RotatedRect rRect = cv::RotatedRect( center,
                                             cv::Size2f( patch->getHeight()*pixelsPerMeter, patch->getWidth()*pixelsPerMeter ),
                                             patch->getPose().getYaw()*180.0/M_PI );

    cv::Point2f vertices[4];
    rRect.points(vertices);
    cv::Scalar pCol = col;//+col*patch->getProbability()*50;
    pCol *= 0.75;
    for (int i = 0; i < 4; i++)
      cv::line(img, vertices[i], vertices[(i+1)%4], pCol, 1 );


  }
  col = cv::Scalar( 0,255,255 );
  for (PatchPtrList::iterator it = env->mCurrentCarPatches.begin();
       it != env->mCurrentCarPatches.end(); it++)
  {
    PatchPtr patch = *it;
    double realX = -x + patch->getPose().getX() + radius;
    double realY = -y + patch->getPose().getY() + radius;
    cv::Point2f center( realX*pixelsPerMeter, realY*pixelsPerMeter );
    cv::RotatedRect rRect = cv::RotatedRect( center,
                                             cv::Size2f( patch->getHeight()*pixelsPerMeter, patch->getWidth()*pixelsPerMeter ),
                                             patch->getPose().getYaw()*180.0/M_PI );

    cv::Point2f vertices[4];
    rRect.points(vertices);
    cv::Scalar pCol = col;//+col*patch->getProbability()*50;
    pCol *= 0.75;
    for (int i = 0; i < 4; i++)
      cv::line(img, vertices[i], vertices[(i+1)%4], pCol, 1 );
  }


  cv::Scalar colStart = cv::Scalar( 0, 0, 255 );
  //cv::Scalar colEnd = cv::Scalar( 235, 206, 135 );
  for( size_t i = 1; i < env->mPastCarPoses.size(); i ++ )
  {
    cv::Point2f point( (-x + env->mPastCarPoses[i].getX() + radius)*pixelsPerMeter,
                       ( -y + env->mPastCarPoses[i].getY() + radius)*pixelsPerMeter );
    cv::Point2f prevPoint( (-x + env->mPastCarPoses[i-1].getX() + radius)*pixelsPerMeter,
        ( -y + env->mPastCarPoses[i-1].getY() + radius)*pixelsPerMeter );
    //float amount = float(i)/float(env->mPastCarPoses.size());
    cv::line( img, prevPoint, point, colStart );//*(1-amount) );// + colEnd*amount );
    cv::circle( img, prevPoint, 2, colStart );//*(1-amount) );// + colEnd*amount );
  }

}


void EnvironmentPainter::drawTrajectory( cv::Mat &img, double x, double y,
                                         double radius, float pixelsPerMeter, Trajectory2d &traj,
                                         cv::Scalar colStart, cv::Scalar colEnd )
{
  //cv::Scalar colStart = cv::Scalar( 0, 64, 255 );
  //cv::Scalar colEnd = cv::Scalar( 128, 255, 128 );
  for( size_t i = 1; i < traj.size(); i ++ )
  {
    cv::Point2f point( (-x + traj[i].getX() + radius)*pixelsPerMeter,
                       ( -y + traj[i].getY() + radius)*pixelsPerMeter );
    cv::Point2f prevPoint( (-x + traj[i-1].getX() + radius)*pixelsPerMeter,
        ( -y + traj[i-1].getY() + radius)*pixelsPerMeter );
    float amount = float(i)/float(traj.size());
    cv::Scalar col;
    cv::add( colStart*float(1.0-amount), colEnd*amount, col );
    cv::line( img, prevPoint, point, col );
    cv::circle( img, prevPoint, 2, col );
  }
}

void EnvironmentPainter::drawGrid( Environment* env, cv::Mat &img, double x, double y,
                                   double radius, float pixelsPerMeter, int imgSize )
{
  // Draw grid:
  float pixelX, pixelY;
  float startX = floor(x - radius);
  for( float gX = startX + 0.5f; gX < x + radius; gX += 1.0f )
  {
    pixelX = (-x + gX + radius)*pixelsPerMeter;
    cv::line( img, cv::Point2f( pixelX, 0 ), cv::Point2f( pixelX, imgSize ), cv::Scalar( 24,24,24 ), 1 );
  }
  for( float gX = startX; gX < x + radius; gX += 1.0f )
  {
    pixelX = (-x + gX + radius)*pixelsPerMeter;
    cv::line( img, cv::Point2f( pixelX, 0 ), cv::Point2f( pixelX, imgSize ), cv::Scalar( 48,48,48 ), 1 );
  }
  float startY = floor(y - radius);
  for( float gY = startY + 0.5f; gY < y + radius; gY += 1.0f )
  {
    pixelY = (-y + gY + radius)*pixelsPerMeter;
    cv::line( img, cv::Point2f( 0, pixelY ), cv::Point2f( imgSize, pixelY ), cv::Scalar( 24,24,24 ), 1 );
  }
  for( float gY = startY; gY < y + radius; gY += 1.0f )
  {
    pixelY = (-y + gY + radius)*pixelsPerMeter;
    cv::line( img, cv::Point2f( 0, pixelY ), cv::Point2f( imgSize, pixelY ), cv::Scalar( 48,48,48 ), 1 );
  }
  if( 0 > startX && 0 < startX + 2*radius )
  {
    pixelX = (-x + radius)*pixelsPerMeter;
    cv::line( img, cv::Point2f( pixelX, 0 ), cv::Point2f( pixelX, imgSize ), cv::Scalar( 32,100,32 ), 1 );
  }
  if( 0 > startY && 0 < startY + 2*radius )
  {
    pixelY = (-y + radius)*pixelsPerMeter;
    cv::line( img, cv::Point2f( 0, pixelY ), cv::Point2f( imgSize, pixelY ), cv::Scalar( 32,32,100 ), 1 );
  }

}

void EnvironmentPainter::drawCar( Environment* env, cv::Mat &img, double x, double y,
                                  double radius, float pixelsPerMeter )
{
  // Draw the car:
  double realX = -x + env->mCar->getPose().getX() + radius;
  double realY = -y + env->mCar->getPose().getY() + radius;
  // Offset the center of the drawn rectangle, because car origin (i.e. the car pose)
  // is not in the center of the car:
  double offsetDist = CAR_ORIGIN_TO_FRONT - env->mCar->getHeight()*0.5;
  double cosCar = cos( env->mCar->getPose().getYaw() );
  double sinCar = sin( env->mCar->getPose().getYaw() );
  double dX = cosCar*offsetDist;
  double dY = sinCar*offsetDist;

  cv::Point2f origin( realX*pixelsPerMeter, realY*pixelsPerMeter );
  cv::Point2f center( (realX + dX)*pixelsPerMeter, (realY + dY)*pixelsPerMeter );
  cv::Point2f dir( cos( env->mCar->getPose().getYaw() ), sin( env->mCar->getPose().getYaw() ) );

  // Find the origin of the camera:
  double camX = mCameraPosY*cosCar - mCameraPosX*sinCar;
  double camY = mCameraPosY*sinCar + mCameraPosX*cosCar;
  cv::Point2f cameraCenter( (realX + camX)*pixelsPerMeter, (realY + camY)*pixelsPerMeter );

  // If there is an active DriverModule given, then draw the steering and speed:
  if( env->mDriver )
  {
    float steerAngle = env->mDriver->getSteeringAngle();
    float globSteerAngle = env->mCar->getPose().getYaw() - steerAngle/180.0*M_PI;

    cv::Scalar col( 200, 200, 200 );
    if( std::abs( steerAngle ) == env->mDriver->getMaxSteeringAngle() )
    {
      col = cv::Scalar( 100, 100, 255 );
    }

    // Draw the steering angle as a circle arc:
    cv::ellipse(img, origin, cv::Size(32,32), 0,
                env->mCar->getPose().getYaw()*180.0/M_PI,
                globSteerAngle*180.0/M_PI,
                col, 1 );
    cv::Point2f dirSteer( cos( globSteerAngle ), sin( globSteerAngle ) );
    cv::line( img, origin, origin + dir*32, col, 1 );
    cv::line( img, origin, origin + dirSteer*32, col , 1 );

    cv::line( img, origin, origin + dir*env->mDriver->getSpeed()*pixelsPerMeter, cv::Scalar( 255,255,255 ), 2 );
    cv::circle( img, origin, 2, cv::Scalar( 255,255,255 ) );
  }

  // Draw the car as a rotated rectangle:
  cv::RotatedRect rRect = cv::RotatedRect( center,
      cv::Size2f( env->mCar->getHeight()*pixelsPerMeter, env->mCar->getWidth()*pixelsPerMeter ),
      env->mCar->getPose().getYaw()*180.0/M_PI );
  cv::Point2f vertices[4];
  rRect.points(vertices);
  for (int i = 0; i < 4; i++)
    cv::line(img, vertices[i], vertices[(i+1)%4], cv::Scalar( 64,255,64 ), 1 );

  // Draw camera:
  rRect = cv::RotatedRect( cameraCenter,
      cv::Size2f( 0.05*pixelsPerMeter, (env->mCar->getWidth()-0.1)*pixelsPerMeter ),
      env->mCar->getPose().getYaw()*180.0/M_PI );
  rRect.points(vertices);
  for (int i = 0; i < 4; i++)
    cv::line(img, vertices[i], vertices[(i+1)%4], cv::Scalar( 255,64,64 ), 1 );

  if( env->mCoordConverter )
  {
    // Bird view image size
    int imageWidth = env->mCoordConverter->getImgSizeBirdView().width;
    int imageHeight = env->mCoordConverter->getImgSizeBirdView().height;

    // Draw the current bird view region:
    cv::Point2f topLeft( 0, 0 );
    ExtendedPose2d topLeftWorld =
        env->mCoordConverter->pixel2World( env->mCar->getPose(), topLeft );

    cv::Point2f topRight( imageWidth, 0 );
    ExtendedPose2d topRightWorld =
        env->mCoordConverter->pixel2World( env->mCar->getPose(), topRight );

    cv::Point2f bottomRight( imageWidth, imageHeight );
    ExtendedPose2d bottomRightWorld =
        env->mCoordConverter->pixel2World( env->mCar->getPose(), bottomRight );

    topLeft.x = pixelsPerMeter*( -x + topLeftWorld.getX() + radius);
    topLeft.y = pixelsPerMeter*( -y + topLeftWorld.getY() + radius);
    topRight.x = pixelsPerMeter*( -x + topRightWorld.getX() + radius);
    topRight.y = pixelsPerMeter*( -y + topRightWorld.getY() + radius);
    bottomRight.x = pixelsPerMeter*( -x + bottomRightWorld.getX() + radius);
    bottomRight.y = pixelsPerMeter*( -y + bottomRightWorld.getY() + radius);

    cv::Point2f centerBirdView = 0.5f*(bottomRight - topLeft) + topLeft;

    float dx = cv::norm( topLeft - topRight );
    float dy = cv::norm( topRight - bottomRight );

    rRect = cv::RotatedRect( centerBirdView, cv::Size( dy, dx ),
                             env->mCar->getPose().getYaw()*180.0f/M_PI );

    rRect.points(vertices);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertices[i], vertices[(i+1)%4], cv::Scalar( 64,64,64 ), 1 );
  }
}

void EnvironmentPainter::drawDebugPoints( Environment* env, cv::Mat &img, double x, double y,
                                          double radius, float pixelsPerMeter )
{
  // Get the delta time since last drawing:
  ptime now = microsec_clock::local_time();
  time_duration dt = now - env->mDebugPointTime;
  env->mDebugPointTime = now;

  // Draw the debug points, if any:
  std::list<DebugPoint>::iterator it;
  for( it = env->mDebugPoints.begin(); it != env->mDebugPoints.end(); )
  {
    (*it).time -= dt;
    if( (*it).time < seconds( 0 ) )
    {
      it = env->mDebugPoints.erase( it );
    } else {
      double realX = -x + (*it).pose.getX() + radius;
      double realY = -y + (*it).pose.getY() + radius;
      cv::Point2f position( realX*pixelsPerMeter, realY*pixelsPerMeter );
      if( it->drawDirection )
      {
        cv::Point2f dir( cos( (*it).pose.getYaw() ), sin( (*it).pose.getYaw() ) );
        cv::line( img, position, position + dir*30, it->col );
        cv::circle( img, position, 2, it->col );
      } else {
        cv::circle(img, position, 2, it->col );
      }
      it++;
    }
  }
}

cv::Mat EnvironmentPainter::getEnvAsImage( Environment* env, double x, double y, double radius,
                                           float pixelsPerMeter )
{
  if( env->mMultiTrajectory.trajectories.size() > 1)
  {
    radius = radius*0.5;
    pixelsPerMeter *= 2;
  }

  int imgSize = 2*radius*pixelsPerMeter+1 ;
  cv::Mat img( imgSize, imgSize, CV_8UC3, cv::Scalar(0,0,0));

  drawGrid( env, img, x, y, radius, pixelsPerMeter, imgSize );

  // draw patches as rectangles:
  drawPatches( env, img, x, y, radius, pixelsPerMeter );

  /*if(mDrawHistory)
  {
    drawHistory( env, img, x, y, radius, pixelsPerMeter );
  }*/
//  drawTrajectory( img,  x,y ,radius,pixelsPerMeter, env->mRawTrajectory,
//                  cv::Scalar( 255, 255, 255 ), cv::Scalar( 0, 128, 128 ) );
  size_t currentIndex = env->mDriver->getCurrentTrajectoryIndex();
  for( size_t i = 0; i < env->mMultiTrajectory.trajectories.size(); i ++ )
  {
    if( i == currentIndex )
    {
      drawTrajectory( img,  x, y, radius, pixelsPerMeter,
          env->mMultiTrajectory.trajectories[i],
          cv::Scalar( 100, 140, 100 ), cv::Scalar( 200, 255, 200 ) );
    } else {
      drawTrajectory( img,  x, y, radius, pixelsPerMeter,
          env->mMultiTrajectory.trajectories[i],
          cv::Scalar( 64, 100, 64 ), cv::Scalar( 64, 128, 64 ) );
    }
  }
  //drawTrajectory( img,  x,y ,radius,pixelsPerMeter, env->mTrajectorySimple,
                  //cv::Scalar( 0, 128, 0 ), cv::Scalar( 0, 64, 0 ) );

  drawCar( env, img, x, y, radius, pixelsPerMeter );

  drawTrafficSign( env, img,x,y,radius,pixelsPerMeter);
  drawObstacles( env, img,x,y,radius,pixelsPerMeter);
  if( mDrawEventRegions )
  {
    drawEventRegions( env, img,x,y,radius,pixelsPerMeter);
  }

  drawDebugPoints( env, img,x,y,radius,pixelsPerMeter);

  cv::Mat flipped( img.size(), CV_8UC3 );
  cv::flip( img, flipped, 0 );

  return flipped;
}

/*std::string EnvironmentPainter::toJson(double radius) {

  std::stringstream ss;

  ss << ""
	"{";

  Environment* env = Environment::getInstance();

  //the car
  EnvObjectPtr car = env->getCar();
  ss << "\"car\": "
      "{"
	"\"type\": \"car\","
	"\"x\": " + boost::lexical_cast<std::string>(car->getX()) + ","
	"\"y\": " + boost::lexical_cast<std::string>(car->getY()) + ","
	"\"yaw\": " + boost::lexical_cast<std::string>(car->getYaw()) +
    "},";

  //Patches
  ss << "\"patches\": [";
  for ( PatchPtrList::iterator it = env->mStreet.begin(); it != env->mStreet.end(); it++ )
  {
      PatchPtr p = *it;
      ss << p->toJson();

      if (it != env->mStreet.end()) {
	  ss << ",";
      }

  }
  ss << "]";

  ss << ""
      "}";

  return ss.str();

}*/
}	// namespace
}	// namespace

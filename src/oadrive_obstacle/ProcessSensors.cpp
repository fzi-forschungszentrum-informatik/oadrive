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
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#include "ProcessSensors.h"
#include "obstacleLogging.h"

#include <oadrive_world/Environment.h>

using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive{
namespace obstacle{

ProcessSensors::ProcessSensors(std::string depthRefImage, std::string usCal, util::CoordinateConverter *converter )
  : mConverter(converter)
    , mProcessDepth(depthRefImage, converter)
    , mProcessUS(usCal)
    , mUsSensorBufferLength( 5 )
  {
  }

void ProcessSensors::processDepthSensor(cv::Mat image)
{
  //setFree(oadrive::world::DEPTH);
  LOGGING_INFO( obstacleLogger, "Processing Depth image." << endl );
  ExtendedPose2dVectorPtr objects = mProcessDepth.getObjects(image);
  mergeObjectsToEnviroment(objects, oadrive::world::DEPTH);
  LOGGING_INFO( obstacleLogger, "\t" << objects->size() << " objects added." << endl );
}

void ProcessSensors::setNewUsSensorValues(usSensor sensorValues)
{
  mtxUs.lock();
  // If the FIFO buffer is full, delete the first element:
  if( mRecentUsSensors.size() >= mUsSensorBufferLength )
  {
    mRecentUsSensors.erase( mRecentUsSensors.begin() );
  }

  // Add the new value to the end of the list:
  mRecentUsSensors.push_back( sensorValues );
  mtxUs.unlock();
}

usSensor ProcessSensors::getQuantilesOfRecentUsSensorValues()
{
  std::vector<float> bufferFrontLeft;
  std::vector<float> bufferFrontCenterLeft;
  std::vector<float> bufferFrontCenter;
  std::vector<float> bufferFrontCenterRight;
  std::vector<float> bufferFrontRight;
  std::vector<float> bufferSideLeft;
  std::vector<float> bufferSideRight;
  std::vector<float> bufferRearLeft;
  std::vector<float> bufferRearCenter;
  std::vector<float> bufferRearRight;

  mtxUs.lock();
  std::vector<usSensor>::iterator it;
  for( it = mRecentUsSensors.begin(); it != mRecentUsSensors.end(); ++ it )
  {
    bufferFrontLeft.push_back( (*it).frontLeft );
    bufferFrontCenterLeft.push_back( (*it).frontCenterLeft );
    bufferFrontCenter.push_back( (*it).frontCenter );
    bufferFrontCenterRight.push_back( (*it).frontCenterRight );
    bufferFrontRight.push_back( (*it).frontRight );
    bufferSideLeft.push_back( (*it).sideLeft );
    bufferSideRight.push_back( (*it).sideRight );
    bufferRearLeft.push_back( (*it).rearLeft );
    bufferRearCenter.push_back( (*it).rearCenter );
    bufferRearRight.push_back( (*it).rearRight );
  }
  mtxUs.unlock();

  usSensor quantiles;
  if( bufferFrontLeft.size() > 0 )
  {
    // Sort the lists to be able to get the median:
    std::sort( bufferFrontLeft.begin(), bufferFrontLeft.end() );
    std::sort( bufferFrontCenterLeft.begin(), bufferFrontCenterLeft.end() );
    std::sort( bufferFrontCenter.begin(), bufferFrontCenter.end() );
    std::sort( bufferFrontCenterRight.begin(), bufferFrontCenterRight.end() );
    std::sort( bufferFrontRight.begin(), bufferFrontRight.end() );
    std::sort( bufferSideLeft.begin(), bufferSideLeft.end() );
    std::sort( bufferSideRight.begin(), bufferSideRight.end() );
    std::sort( bufferRearLeft.begin(), bufferRearLeft.end() );
    std::sort( bufferRearCenter.begin(), bufferRearCenter.end() );
    std::sort( bufferRearRight.begin(), bufferRearRight.end() );

    // Retrieve the median values:
    unsigned int quantileIndex = floor( bufferFrontLeft.size()/2 );
    quantiles.frontLeft = bufferFrontLeft[quantileIndex];
    quantiles.frontCenterLeft = bufferFrontCenterLeft[quantileIndex];
    quantiles.frontCenter = bufferFrontCenter[quantileIndex];
    quantiles.frontCenterRight = bufferFrontCenterRight[quantileIndex];
    quantiles.frontRight = bufferFrontRight[quantileIndex];
    quantiles.sideLeft = bufferSideLeft[quantileIndex];
    quantiles.sideRight = bufferSideRight[quantileIndex];
    quantiles.rearLeft = bufferRearLeft[quantileIndex];
    quantiles.rearCenter = bufferRearCenter[quantileIndex];
    quantiles.rearRight = bufferRearRight[quantileIndex];
  } else {
    float maxValue = 10.0;
    quantiles.frontLeft = maxValue;
    quantiles.frontCenterLeft = maxValue;
    quantiles.frontCenter = maxValue;
    quantiles.frontCenterRight = maxValue;
    quantiles.frontRight = maxValue;
    quantiles.sideLeft = maxValue;
    quantiles.sideRight = maxValue;
    quantiles.rearLeft = maxValue;
    quantiles.rearCenter = maxValue;
    quantiles.rearRight = maxValue;
  }

  return quantiles;
}

void ProcessSensors::processUsSensor()
{
  usSensor quantiles = getQuantilesOfRecentUsSensorValues();

  LOGGING_INFO(obstacleLogger, "quantiles:" << endl
      << "\tfrontLeft: " << quantiles.frontLeft << endl
      << "\tfrontCenterLeft: " << quantiles.frontCenterLeft << endl
      << "\tfrontCenter: " << quantiles.frontCenter << endl
      << "\tfrontCenterRight: " << quantiles.frontCenterRight << endl
      << "\tfrontRight: " << quantiles.frontRight << endl
      << "\tsideLeft: " << quantiles.sideLeft << endl
      << "\tsideRight: " << quantiles.sideRight << endl
      << "\trearLeft: " << quantiles.rearLeft << endl
      << "\trearCenter: " << quantiles.rearCenter << endl
      << "\trearRight: " << quantiles.rearRight << endl);

  ExtendedPose2dVectorPtr objects = mProcessUS.getObjects(quantiles);

  convertObjectsToWorld(objects, Environment::getInstance()->getCarPose());
  mergeObjectsToEnviroment(objects, oadrive::world::US);
  LOGGING_INFO(obstacleLogger, "Merged objects.");
}

void ProcessSensors::mergeObjectsToEnviroment(ExtendedPose2dVectorPtr objects, oadrive::world::SensorType sensorType)
{
  oadrive::world::ObstaclePtr myObstaclePtr;
  LOGGING_INFO( obstacleLogger, "Adding " << objects->size() << " objects to environment." << endl <<
      "\tSensor type: " << sensorType << endl );
  for( unsigned int i = 0; i< (*objects).size(); i++ )
  {
    /*myObstaclePtr = mEnvironment->getNearestObstacle(objects->at(i));
      if(myObstaclePtr){
    //            std::cerr<<"found object"<<myObstaclePtr->getPose()<<std::endl;
    if(myObstaclePtr->calcDistTo(objects->at(i))>MAXDISTTOENVOBJECT)
    {
    //object isn't in the Enviroment add it
    //double size = 0;
    //                if(sensorType == US)
    //                {
    //                    size = 0.05;
    //                }
    //                else
    //                {
    //                    size = 0.10;
    //                }
    //size = 0.05;
    oadrive::world::ObstaclePtr newObject(new oadrive::world::Obstacle(objects->at(i),0.05,0.05,sensorType));
    mEnvironment->addObstacle(newObject);
    LOGGING_INFO(obstacleLogger,"Add object to Environment Pos: x:"<<objects->at(i).getX()<<objects->at(i).getY()<<"Dist to nearest Obj.: "<<myObstaclePtr->calcDistTo(objects->at(i))<<endl);
    }
    else
    {
    LOGGING_INFO(obstacleLogger,"Udate object in Environment with Pos: x: "<<objects->at(i).getX()
    << " y: " <<objects->at(i).getY()<<endl);
    myObstaclePtr->update(objects->at(i),sensorType);
    }
    }
    else //there are no objects
    {*/
    oadrive::world::ObstaclePtr newObject(new oadrive::world::Obstacle(objects->at(i),0.05,0.05,sensorType));
    Environment::getInstance()->addObstacle(newObject);
    //}
  }
}

void ProcessSensors::setFree(SensorType sensor)
{
  /*cv::Size imgSize = mConverter->getImgSizeBirdView();
    ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
    oadrive::core::ExtendedPose2d birdviewCenter =
    mConverter->pixel2World( carPose ,cv::Point2d(320,240));
    birdviewCenter.setYaw( carPose.getYaw() );
    double meterPerPixel = mConverter->getMetersPerPixel();
    double birdViewWidth = meterPerPixel*imgSize.width;
    double birdViewLength = meterPerPixel*imgSize.height;
    oadrive::world::EnvObjectPtr Birdview(new  oadrive::world::EnvObject(birdviewCenter,birdViewWidth,birdViewLength));
    oadrive::world::ObstaclePtrList obstaclesList =
    Environment::getInstance()->getObstaclesCrossEnvObject(Birdview);
    for(ObstaclePtrList::iterator it = obstaclesList.begin();it != obstaclesList.end();it++)
    {
    ObstaclePtr obstacle = *it;
    obstacle->setFree(sensor);
    }*/


}

void ProcessSensors::convertObjectsToWorld(ExtendedPose2dVectorPtr &objects,const oadrive::core::ExtendedPose2d &carPos)
{
  for(unsigned int i = 0; i<objects->size();i++)
  {
    objects->at(i) = mConverter->car2World(carPos,objects->at(i));
  }
}


}
}

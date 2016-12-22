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

#ifndef OADRIVE_OBSTACLES_PROCESSUSSENSOR_H
#define OADRIVE_OBSTACLES_PROCESSUSSENSOR_H
#include <opencv2/core/core.hpp>
#include <oadrive_obstacle/ProcessUS.h>
#include <vector>
#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_world/Obstacle.h>
#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_obstacle/ProcessDepth.h>
namespace oadrive{
namespace obstacle{

class ProcessSensors
{
public:
  ProcessSensors(std::string depthRefImage,std::string usCal, util::CoordinateConverter *converter);
  //! \brief ProcessDepthSensor Processes the Depth image and add found objects to the enviroment
  //! \param image depthimage
  void processDepthSensor(cv::Mat image);

  void setNewUsSensorValues(usSensor sensorValues);
  void processUsSensor();
  void setUsSensorBufferLengtg( int length ) { mUsSensorBufferLength = length; }
  usSensor getQuantilesOfRecentUsSensorValues();
  ProcessDepth* getDepthImageProcessor(){return &mProcessDepth;}
  void mergeObjectsToEnviroment(ExtendedPose2dVectorPtr objects, oadrive::world::SensorType sensorType);
  void convertObjectsToWorld(ExtendedPose2dVectorPtr &objects, const ExtendedPose2d &carPos);

  void setFree(oadrive::world::SensorType sensor);

private:

  oadrive::util::CoordinateConverter *mConverter;
  ProcessDepth mProcessDepth;
  ProcessUS mProcessUS;

  //! FIFO buffer for ultrasonic values:
  std::vector<usSensor> mRecentUsSensors;
  size_t mUsSensorBufferLength;

  boost::mutex mtxUs;   // Mutex lock for US Sensors

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
}
}

#endif // OADRIVE_OBSTACLES_PROCESSUSSENSOR_H

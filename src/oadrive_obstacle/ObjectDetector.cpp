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
 * \author  Robin Andlauer <robin.andlauer@student.kit.edu>
 * \date    2017-8-25
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Shuxiao Ding <ding@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------
#include "ObjectDetector.h"

#define VERBOSE

namespace oadrive{
namespace obstacle{

ObjectDetector::ObjectDetector(std::string birdViewConfigPath)
  : mBirdViewConverter()
  , mCoordinateConverter(birdViewConfigPath)
  , mBoundingBoxPixelOffsetY(0)
  , mX_measurementCovariance_ego((float)oadrive::util::Config::getDouble( "ObjectDetector", "X_measurementCovariance_ego", 0.5))
  , mY_measurementCovariance_ego((float)oadrive::util::Config::getDouble( "ObjectDetector", "Y_measurementCovariance_ego", 0.5))
  , mWidthMeasurementCovariance((float)oadrive::util::Config::getDouble( "ObjectDetector", "WidthMeasurementCovariance", 0.5))
{
  mBirdViewConverter.loadConfig(birdViewConfigPath);
}

ObjectDetector::~ObjectDetector()
{

}

DetectedObjects ObjectDetector::findObjects(std::vector<BoundingBox> boundingBoxes, ExtendedPose2d egoCarPose_odo)
{
  DetectedObjects objects;

  for(auto & object : boundingBoxes)
  {
    if(strcmp(object.Class.c_str(),"car") == 0)
    {
      if(object.ymax < 400) // if ymax > 950 YOLO most likely detected the car picture on the car's own case -.-
      {
        // store bottom points of the bounding box. This bounding box is usually estimated too large
        // Hence, a certain number of pixels are substracted from the y-coordinates.
        std::vector<cv::Point2f> boxPixels;
        boxPixels.push_back(cv::Point2f(object.xmin,object.ymax-mBoundingBoxPixelOffsetY)); //bottomLef -mBoundingBoxPixelOffsetY
        boxPixels.push_back(cv::Point2f(object.xmax,object.ymax-mBoundingBoxPixelOffsetY)); //bottomRight

        DetectedObject object;
        if (this->estimateObjectFromYOLO(object, boxPixels, egoCarPose_odo)) {
          DetectedCar car;
          car.object = object;
          objects.cars.push_back(car);
        }
//      DetectedCar car = estimateCarPoseFromBirdView(boxPixels, egoCarPose_odo);
//      objects.cars.push_back(car);

//      // debug: publish box points as persons
//      std::vector<cv::Point2f> boxBirdViewPixels = mBirdViewConverter.transformPixels(boxPixels);
//      ExtendedPose2d p_1 = mCoordinateConverter.pixel2Car(boxBirdViewPixels[0]);
//      DetectedPerson p1 = {0,0,p_1,0};
//      ExtendedPose2d p_2 = mCoordinateConverter.pixel2Car(boxBirdViewPixels[1]);
//      DetectedPerson p2 = {0,0,p_2,0};
//      objects.persons.push_back(p1);
//      objects.persons.push_back(p2);
      }
    }
    else if(strcmp(object.Class.c_str(), "person") == 0) {
      std::vector<cv::Point2f> boxPixels;
      boxPixels.push_back(cv::Point2f(object.xmin, object.ymax-mBoundingBoxPixelOffsetY));
      boxPixels.push_back(cv::Point2f(object.xmax, object.ymax-mBoundingBoxPixelOffsetY));

      float rate = (float) (object.ymax - object.ymin) / (object.xmax - object.xmin);

      DetectedObject object;
      if (this->estimateObjectFromYOLO(object, boxPixels, egoCarPose_odo)) {
        DetectedPerson person;
        person.object = object;
        person.height = person.object.width * rate;
        person.heightVariance = person.object.widthVariance * rate;
        objects.persons.push_back(person);
      }

    }

  }

  return objects;
}

bool ObjectDetector::estimateObjectFromYOLO(DetectedObject &object, const std::vector<cv::Point2f> boxPixels, const ExtendedPose2d egoCarPose_odo)
{
  // transform box corners to birdview
  std::vector<cv::Point2f> boxBirdViewPixels = mBirdViewConverter.transformPixels(boxPixels);

  // sanity checks:
  if (!mCoordinateConverter.isPixel2CarCorrect(boxBirdViewPixels[0]) || !mCoordinateConverter.isPixel2CarCorrect(boxBirdViewPixels[0])) {
    return false;
  }
  // convert pixels to coordinates in ego-frame
  ExtendedPose2d bottomLeft_ego = mCoordinateConverter.pixel2Car(boxBirdViewPixels[0]);
  ExtendedPose2d bottomRight_ego = mCoordinateConverter.pixel2Car(boxBirdViewPixels[1]);
  float dx = bottomRight_ego.getX() - bottomLeft_ego.getX();
  float dy = bottomRight_ego.getY() - bottomLeft_ego.getY();
  object.width = sqrt(dx * dx + dy * dy);

  // get the center of the bottom line of the box
  cv::Point2f bottomCenterPixel((boxBirdViewPixels[0].x+boxBirdViewPixels[1].x)/2,(boxBirdViewPixels[0].y+boxBirdViewPixels[1].y)/2);
  ExtendedPose2d detectedObject_ego = mCoordinateConverter.pixel2Car(bottomCenterPixel);
  this->estimateObjectMeasurementVariance(object, egoCarPose_odo, detectedObject_ego);
  object.pose = mCoordinateConverter.car2World(egoCarPose_odo, detectedObject_ego);

  return true;
}



void ObjectDetector::estimateObjectMeasurementVariance(DetectedObject &object, const ExtendedPose2d egoCarPose_odo, const ExtendedPose2d  detectedObject_ego)
{
  float x = detectedObject_ego.getX();
  float y = detectedObject_ego.getY();
  float objectToCarDistance = sqrt( x * x + y * y);

  // cap distance
  objectToCarDistance = (objectToCarDistance < 1) ? 1 :
                        (objectToCarDistance > 3) ? 3 :
                         objectToCarDistance;

  // calculate camera angle to object
  float objectToCameraAngle = fabs(atan2(y,x-0.3));
//  LOGGING_INFO(obstacleLogger, "[obstacleLogger] Angle: " << objectToCameraAngle  << " in degrees: " << objectToCameraAngle*180/M_PI<< endl);

  // cap angle
  objectToCameraAngle = (objectToCameraAngle < 35) ? 35 :
                        (objectToCameraAngle > 45) ? 45 :
                         objectToCameraAngle;

//  LOGGING_INFO(obstacleLogger, "[obstacleLogger] Factor: " << (pow(10.0f,objectToCarDistance-1) + pow(10.0f,(objectToCameraAngle-35)/5.0f)) / 2 << endl);
  // multiply covariance by distance and angle (each term between 1 and 100)

  float varianceFactor = (pow(10.0f,objectToCarDistance-1) + pow(10.0f,(objectToCameraAngle-35)/5.0f)) / 2;
  ExtendedPose2d poseVariance_ego(mX_measurementCovariance_ego * varianceFactor, mY_measurementCovariance_ego * varianceFactor, 0);

  // convert to global variance. Not rly tested if this works as intended
  object.poseVariance.setX(fabs(poseVariance_ego.getY()*sin(egoCarPose_odo.getYaw())+poseVariance_ego.getX()*cos(egoCarPose_odo.getYaw())));
  object.poseVariance.setY(fabs(poseVariance_ego.getX()*sin(egoCarPose_odo.getYaw())+poseVariance_ego.getY()*cos(egoCarPose_odo.getYaw())));

  object.widthVariance = mWidthMeasurementCovariance;
 // std::cout << "poseVariance" << object.poseVariance.getX() << " " << object.poseVariance.getY() << std::endl;
}

}
}

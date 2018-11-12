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
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_OBSTACLE_OBJECTDETECTOR_H
#define OADRIVE_OBSTACLE_OBJECTDETECTOR_H
/*!
   \brief Detects and classifies objects using YOLO and depth and publishes them by sending them to a ros node
 */
#include "oadrive_util/BirdViewConverter.h"
#include "oadrive_util/CoordinateConverter.h"
#include "oadrive_core/ExtendedPose2d.h"
#include "oadrive_util/Config.h"
#include "oadrive_obstacle/obstacleLogging.h"


/*
 * coordinate frames:
 *      _ego: The origin is between the car's rear wheels (center of the rear axis).
 *              X increases towards the front of the car, Y increases towards the left of the car.
 *      _odo: global coordinates determined via the car position from the odometrie only
 *              (coordinates only reliable near the current position of the car)
 *      _world: global coordinates determined via the car position from the odometrie AND sign synchronisation
 *              (coordinates globally reliable through synchronistation every time a shield is detected)
 */
using icl_core::logging::endl;
using icl_core::logging::flush;
using namespace oadrive::core;

namespace oadrive{
namespace obstacle{

struct BoundingBox
{
  std::string Class;
  float probability;
  int xmin, ymin;
  int xmax, ymax;
};

struct DetectedObject
{
  ExtendedPose2d pose;
  ExtendedPose2d poseVariance;
  float width;
  float widthVariance;
};

struct DetectedCar
{
  DetectedObject object;
};

struct DetectedPerson
{
  DetectedObject object;
  float height;
  float heightVariance;
};

struct DetectedObjects
{
  std::vector<DetectedCar> cars;
  std::vector<DetectedPerson> persons; //appearently the word "persons" exists
};


//!
//! \brief The ObjectDetector class detects/ calculates the positon + variance of objects
//! Coordinate system abbreviations used in this class:
//! _odo: unsynchronized world coordinates (measured via odometrie but without using traffic signs);
//! _ego: local car pose with the origin at the back wheel of the car. +X in car direction; +Y perpendicular to the left
//!
class ObjectDetector
{
  public:
    //!
    //! \brief ObjectDetector
    //! \param birdViewConfigPath Path to config file with camera matrix
    //!
    ObjectDetector(std::string birdViewConfigPath);

    ~ObjectDetector();
    //!
    //! \brief findObjects reads the bounding boxes from YOLO and calculates width, global position and variance
    //! \param boundingBoxes all bounding boxes found in one image
    //! \param egoCarPose_odo global car pose
    //! \return detected objects found in one image containing cars and people
    //!
    DetectedObjects findObjects(std::vector<BoundingBox> boundingBoxes, ExtendedPose2d egoCarPose_odo);

  private:
    //!
    //! \brief estimateObjectFromYOLO calculates position+width of object and calls estimateObjectMeasurementVariance
    //! \param object referenced object where the pose etc. is saved
    //! \param boxPixels bottom corners of the bounding box in pixel coordinates
    //! \param egoCarPose_odo global car pose
    //!
    bool estimateObjectFromYOLO(DetectedObject &object, const std::vector<cv::Point2f> boxPixels, const ExtendedPose2d egoCarPose_odo);
    //!
    //! \brief estimateObjectMeasurementVariance estimates measurement variance. Objects that are further away or
    //! adjacent to the image borders are weighted with a higher variance
    //! \param object object where the variance will be saved
    //! \param egoCarPose_odo global car pose
    //! \param detectedObject_ego position of the object in local coordinates
    //!
    void estimateObjectMeasurementVariance(DetectedObject &object, const ExtendedPose2d egoCarPose_odo, const ExtendedPose2d  detectedObject_ego);

    //! converter to transform pixel to local coordinates
    oadrive::util::BirdViewConverter mBirdViewConverter;
    //! converter to transform local coordinates to global
    oadrive::util::CoordinateConverter mCoordinateConverter;

    //! variable could be used, if the bounding boxes have a constant offset from the "ground truth"
    const unsigned int mBoundingBoxPixelOffsetY;
    //! variance in local x direction
    const float mX_measurementCovariance_ego;
    //! variance in local y direction
    const float mY_measurementCovariance_ego;
    //! variance for the width
    const float mWidthMeasurementCovariance;
};

}
}
#endif


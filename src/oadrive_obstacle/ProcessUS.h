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
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_OBSTACLE_PROCESSUS_H
#define OADRIVE_OBSTACLE_PROCESSUS_H
#include <opencv2/core/core.hpp>
#include "string.h"
#include <iostream>
#include "time.h"
#include <oadrive_core/ExtendedPose2d.h>
#include <boost/shared_ptr.hpp>
#include <vector>

#define NUMBERCALPOINTSPERSENSOR 2
#define NUMBERSENSOR 10

namespace oadrive{
namespace obstacle{

struct usSensor {
  float frontLeft;
  float frontCenterLeft;
  float frontCenter;
  float frontCenterRight;
  float frontRight;
  float sideLeft;
  float sideRight;
  float rearLeft;
  float rearCenter;
  float rearRight;
};

enum enumUSSensorLimits {
  LIMIT_FOR_CROSSING,
  LIMIT_FOR_SEARCH_PARKING,
  LIMIT_FOR_OVERTAKING,
  LIMIT_FOR_DRIVING,
  LIMIT_FOR_PED_CROSSING,
  LIMIT_DEACTIVATE
};
/*!
   \brief The ProcessUS class converts the distances from the US-Sensors to car postions. You need a config file to use this class. You can generate this file with getValuesFromCons and saveCal
 */
class ProcessUS
{
public:
  ProcessUS();
  //!\param calFile path to cal file were the postion of the US sensors is stored
  ProcessUS(std::string calFile);
  /*!
    * \brief loadCalPoints Loads the calibration point. Use UltrasonicCal to write the points
    * \param path Path for loading the calibraten points
    */
  void loadCalPoints(std::string path);
  /*!
    \brief transform the US Sensor to the car grid
    \param sensorNumber Number of the sensor start at 0 at the left of the frontbumber. Count clockwise
    \return position of the objekt in Car Grid
    */
  core::ExtendedPose2d transformToCar(int sensorNumber, double distance);

  //! \brief getObjects transforms every US sensor in one object
  //! \param sensor struct with all distances in m
  //! \return boost shared Pointer to vector with Extended Poses
  core::ExtendedPose2dVectorPtr getObjects(usSensor sensor);

  /*!
   * \brief printCalPoints Print Cal Points to console
   */
  void printCalPoints();
  //!print sensor positions to console
  void printSensorPos();
  //!Save all calibration points
  void saveCal(std::string path);
  //!Set one Calibration Point
  /*!
   * \brief setCalPoint
   * \param sensorNumber Number of US sensor(beginn at 0 at the left of thr front bumber. Count clockwise)
   * \param x x-coordinates of calibration point in the car grid
   * \param y y-coordinates of calibration point in the car grid
   * \param d distance from calibration point to US sensor
   * \param calPointNumber Number of calibration Point. Two points are needed for each sensor
   */
  void setCalPoint(int sensorNumber, double x, double y, double d, int calPointNumber);
  /*!
   * \brief getValuesFromCons show consule Prompt for setting cal points
   */
  void getValuesFromCons(void);

private:

  /*! calcSensorPos Calculate Sensorpostions in the car. Writes to mUsSensorPos */
  void calcSensorPos();


  struct structCalPoint{
    double distance;
    double x;
    double y;
  };
  //!holds each calibration point loaded from the file
  structCalPoint mCalPoints[NUMBERSENSOR][NUMBERCALPOINTSPERSENSOR];


  /*!
   * \brief The structUsSensorPos struct holds the postion of the Sensor
   *
   * The position is stored in an speacial format so it is easy to calclulate the postion of the objects
   *
   * ______
   * |    /
   * |   /_______
   * |  /        |
   * | / angle    distanceOffset (look at the hypotenuse)
   * |/__________|
   * |           |
   * |            y
   * |___________|
   *
   */
  struct structUsSensorPos{
    double yOffset;
    double angle;
    double distanceOffset;
  };

  /*! mUsSensorPos holds the postion of the US Sensors */
  structUsSensorPos mUsSensorPos[NUMBERSENSOR];
  const double MAXDIST;
  const double MINDIST;
public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};

}	// namespace
}	// namespace

#endif // OADRIVE_OBSTACLE_PROCESSUS_H

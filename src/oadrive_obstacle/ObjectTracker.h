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
 * \date    2017-8-31
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Shuxiao Ding <ding@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_OBSTACLE_OBJECTTRACKER_H
#define OADRIVE_OBSTACLE_OBJECTTRACKER_H
/*!
   \brief Assign measurements to objects by nearest neighbor and creates, deletes or calls a calman filter for each objects
 */

#include "oadrive_core/ExtendedPose2d.h"
#include "oadrive_obstacle/ObjectDetector.h"
#include "oadrive_obstacle/KalmanFilter.h"
#include "oadrive_obstacle/obstacleLogging.h"
#include "oadrive_util/Config.h"
#include "oadrive_util/CoordinateConverter.h"
#include "oadrive_world/TrafficSign.h"

using icl_core::logging::endl;
using icl_core::logging::flush;
using namespace oadrive::core;
using namespace oadrive::util;
using namespace oadrive::world;

namespace oadrive{
namespace obstacle{

struct TrackedObject
{
  ExtendedPose2d pose;
  Eigen::MatrixXf poseErrorCovariance;
  float width;
  float widthErrorCovariance;
  unsigned int filterID;
  unsigned int numberOfMeasurements;

  float calcDistTo(const ExtendedPose2d &other) const {
    return std::sqrt(std::pow(pose.getX() - other.getX(), 2) + std::pow(pose.getY() - other.getY(), 2));
  }
};

struct TrackedCar
{
  TrackedObject object;
  ExtendedPose2d speed;
};

struct TrackedPerson
{
  TrackedObject object;
  ExtendedPose2d speed;
  float height;
  float heightErrorCovariance;
};

struct TrackedObjects
{
  std::vector<TrackedCar> cars;
  std::vector<TrackedPerson> persons;
  std::vector<TrackedObject> obstacles;
  std::vector<TrafficSign> trafficSigns;
  int follow_car_id;
};

struct ConstValue : SystemModel
{
  ConstValue(float q) : spectralPowerDensity(q){}

  MatrixXf stateMatrix(float dt) const
  {
    Eigen::MatrixXf Phi(1,1);
    Phi << 1.0f;
    return Phi;
  }

  MatrixXf systemNoiseCovarianceMatrix(float dt) const
  {
    Eigen::MatrixXf Q(1,1);
    Q << dt*spectralPowerDensity;
    return Q;
  }

  MatrixXf outputMatrix() const
  {
      Eigen::MatrixXf H(1, 1);
      H << 1.0f;
      return H;
  }

  float spectralPowerDensity;
};

struct ConstValue2d: SystemModel
{
  ConstValue2d(float q) : spectralPowerDensity(q){}
  //! state_vector x := [x y v_x v_y] with v_x being the velocity in x-direction and x,y being the position
  //! discrete system matrix:
  MatrixXf stateMatrix(float dt) const
  {
    Eigen::MatrixXf Phi(2, 2);
    Phi << 1, 0,
           0, 1;
    return Phi;
  }
  //! system noise covariance matrix: see paper referenced in the documentation
  MatrixXf systemNoiseCovarianceMatrix(float dt) const
  {
    Eigen::MatrixXf Q(2, 2);
    Q << dt, 0,
         0, dt;
    Q *= spectralPowerDensity;
    return Q;
  }
  //! the output is only the position since we only measure position
  MatrixXf outputMatrix() const
  {
      Eigen::MatrixXf H(2, 2);
      H << 1, 0,
           0, 1;
      return H;
  }
  float spectralPowerDensity;
};

struct ConstValue4d: SystemModel
{
  ConstValue4d(float q) : spectralPowerDensity(q){}
  //! state_vector x := [x y v_x v_y] with v_x being the velocity in x-direction and x,y being the position
  //! discrete system matrix:
  MatrixXf stateMatrix(float dt) const
  {
    Eigen::MatrixXf Phi(4, 4);
    Phi << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    return Phi;
  }
  //! system noise covariance matrix: see paper referenced in the documentation
  MatrixXf systemNoiseCovarianceMatrix(float dt) const
  {
    Eigen::MatrixXf Q(4, 4);
    Q << dt, 0, 0, 0,
         0, dt, 0, 0,
         0, 0, dt, 0,
         0, 0, 0, dt;
    Q *= spectralPowerDensity;
    return Q;
  }
  //! the output is only the position since we only measure position
  MatrixXf outputMatrix() const
  {
      Eigen::MatrixXf H(2, 4);
      H << 1, 0, 0, 0,
           0, 1, 0, 0;
      return H;
  }
  float spectralPowerDensity;
};


struct ConstVelocity : SystemModel
{
  ConstVelocity(float q) : spectralPowerDensity(q){}
  //! state_vector x := [x y v_x v_y] with v_x being the velocity in x-direction and x,y being the position
  //! discrete system matrix:
  MatrixXf stateMatrix(float dt) const
  {
    Eigen::MatrixXf Phi(4, 4);
    Phi << 1, 0, dt, 0,
           0, 1, 0, dt,
           0, 0, 1, 0,
           0, 0, 0, 1;
    return Phi;
  }
  //! system noise covariance matrix: see paper referenced in the documentation
  MatrixXf systemNoiseCovarianceMatrix(float dt) const
  {
    Eigen::MatrixXf Q(4, 4);
    Q << 1/3*pow(dt,3.0f), 0, 1/2*pow(dt,2.0f), 0,
         0, 1/3*pow(dt,3.0f), 0, 1/2*pow(dt,2.0f),
         1/2*pow(dt,2.0f), 0, dt, 0,
         0, 1/2*pow(dt,2.0f), 0, dt;
    Q *= spectralPowerDensity;
    return Q;
  }
  //! the output is only the position since we only measure position
  MatrixXf outputMatrix() const
  {
      Eigen::MatrixXf H(2, 4);
      H << 1, 0, 0, 0,
           0, 1, 0, 0;
      return H;
  }
  float spectralPowerDensity;
};

struct ObstacleFilter
{
  std::vector<ExtendedPose2d> poseList;
  ExtendedPose2d pose;
  unsigned int filterID;
  time_t lastMeasurementTime;
};

struct ObjectFilter
{
  std::shared_ptr<KalmanFilter> poseFilter;
  std::shared_ptr<KalmanFilter> widthFilter;
  unsigned int filterID;
  unsigned int numberOfMeasurements;
  time_t lastMeasurementTime;
};

struct TrafficSignFilter
{
  TrafficSignFilter(TrafficSign t) : trafficSign(t) {};
  std::vector<ExtendedPose2d> poseList; // last positions will be averaged
  TrafficSign trafficSign;
  time_t lastMeasurementTime;
};

struct CarFilter : ObjectFilter
{

};

struct PersonFilter : ObjectFilter
{
  std::shared_ptr<KalmanFilter> heightFilter;
};

//!
//! \brief The ObjectTracker class associated, creates or deletes filters to track the objects using Kalman-Filters
//! Coordinate system abbreviations used in this class:
//! _odo: unsynchronized world coordinates (measured via odometrie but without using traffic signs);
//! _ego: local car pose with the origin at the back wheel of the car. +X in car direction; +Y perpendicular to the left
//!
class ObjectTracker
{
  public:
    //!
    //! \brief ObjectTracker
    //! \param coordinateConverter used for transformations between local and global car psose
    //!
    ObjectTracker(CoordinateConverter *coordinateConverter);
    ~ObjectTracker();

    //!
    //! \brief trackObjects reads all detected objects and starts the processing chain
    //! \param detectedObjects detected objects in one image
    //!
    void trackObjects(const DetectedObjects &detectedObjects);

    //!
    //! \brief getTrackedObjects checks the existing filters and gives back all tracked objects
    //! \param carPose_odo global car position used to check wether all filters are still within the ROI
    //! \return return all currently tracked objects
    //!
    TrackedObjects getTrackedObjects(const ExtendedPose2d &carPose_odo);

    // void trackEmergencyBrakeObstacle(bool emergencyBrake, std::vector<int> US_id, std::vector<float> US_distance);

    //!
    //! \brief trackUS_obstacle tracks an obstacle found by the us sensors.
    //! \param US_position estimated position of the obstacle
    //!
    void trackUS_obstacle(std::vector<ExtendedPose2d> US_position);
    //!
    //! \brief trackTrafficSign reads in the detected traffic signs and associated them with already tracked signs/ creates a new traffic sign
    //! \param trafficSign detected traffic sign
    //!
    void trackTrafficSign(oadrive::world::TrafficSign trafficSign);
    //! resets object tracker i.e. all filters are deleted
    void reset();

  private:

    //!
    //! \brief trackCars associated detected cars with existing filters or creates new filters. Then, the Kalman filter is updated by the new position
    //! \param detectedCars
    //!
    void trackCars(std::vector<DetectedCar> detectedCars);

    void trackPersons(std::vector<DetectedPerson> detectedPersons);

    //! write back the specific objects by checking all filters
    std::vector<TrackedCar> getTrackedCars();
    std::vector<TrackedPerson> getTrackedPersons();
    std::vector<TrackedObject> getTrackedObstacles(); 
    std::vector<TrafficSign> getTrackedTrafficSigns();

    //!
    //! \brief getCarFollowingId determines wether we should follow any car and return its filter ID.
    //! It should return an id if the car is in front of the car and within X meters range
    //! Only tested on rosbags
    //! \param carPose_odo global car pose
    //! \param trackedCars list of all tracked cars
    //! \return filter id of the tracked car we should follow
    //!
    int getCarFollowingId(const ExtendedPose2d &carPose_odo, const std::vector<TrackedCar> &trackedCars);

    //!
    //! \brief updateCars updates the Kalman filters with the new car detections based on the nearest neighbor association
    //! \param detectedCars
    //! \param measurementID_forEachCar Each car filter is assigned one measurement ID. If no measurement was associated to the car filter, the id is -1
    //!
    void updateCars(std::vector<DetectedCar> detectedCars, std::vector<int> measurementID_forEachCar);

    void updatePersons(std::vector<DetectedPerson> detectedPersons, std::vector<int> measurementID_forEachPerson);
    //!
    //! \brief checkAndDeleteObjects checks if the tracked object filters are outside ROI or havent been detected for X seconds. If so, delete the filter
    //! \param carPose_odo global car pose
    //!
    void checkAndDeleteObjects(const ExtendedPose2d &carPose_odo);
    //!
    //! \brief nearestNeighborDataAssociation if within the gate, each car filter is associated with exactly 1 or 0 measurements/detections
    //! \param mahalanobisDistances matrix of the calculated mahalanobis distances between measurement and filters.
    //! Each row contains a different measurement. Each column a different filter
    //! \return vector with the measurement id for every filter. -1 if no measurement was found for given filter
    //!
    std::vector<int> nearestNeighborDataAssociation(std::vector<std::vector< float > > mahalanobisDistances);

    //! \brief createCarFilter create a new car filter
    CarFilter createCarFilter(DetectedCar detectedCar);
    PersonFilter createPersonFilter(DetectedPerson detectedPerson);
    //! average position over the given position list
    ExtendedPose2d averagePoses(std::vector<ExtendedPose2d> poseList);

    CoordinateConverter* mCoordinateConverter;
    std::vector<CarFilter> mTrackedCars;
    std::vector<PersonFilter> mTrackedPersons;
    std::vector<ObstacleFilter> mTrackedObstacles;
    std::vector<TrafficSignFilter> mTrackedTrafficSigns;
//    ExtendedPose2d mCarPose_odo;
    //! tuning parameter for kalman filter -> process/system noise
    const float mCarSpectralPowerDensity;
    const float mPersonSpectralPowerDensity;
    //! only measurements within the gate (mahalanobis distanz) will be considered for NN association
    const float mCarGate;
    const float mPersonGate;
    //! if measurement lies outside this gate for all filters, a new filter is created. --> Could be merged with carGate
    const float mCarGateForNewFilter;
    const float mPersonGateForNewFilter;
    //! Region of interest around the car
    cv::Rect mROI;
    //! if no measurement was found for the given car/obstacle filter over mMaxTimeBeforeDeletion, the filter is deleted
    const float mMaxTimeBeforeDeletionCar;
    const float mMaxTimeBeforeDeletionSign;
    const float mMaxTimeBeforeDeletionPerson;
    const float mMaxTimeBeforeDeletionObstacle;
    //! filter ID counter. Incremented every time a filter is created. Only resets if objectTracker is restarted
    unsigned int mID_counter;
};

}
}

#endif


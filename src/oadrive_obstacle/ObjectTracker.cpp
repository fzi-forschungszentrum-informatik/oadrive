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
#include "ObjectTracker.h"

namespace oadrive{
namespace obstacle{

ObjectTracker::ObjectTracker(oadrive::util::CoordinateConverter* coordinateConverter)
  : mCoordinateConverter(coordinateConverter)
  , mCarSpectralPowerDensity((float)oadrive::util::Config::getDouble( "ObjectTracker", "CarSpectralPowerDensity", 0.5))
  , mPersonSpectralPowerDensity((float)oadrive::util::Config::getDouble( "ObjectTracker", "PersonSpectralPowerDensity", 0.5))
  , mCarGate((float)oadrive::util::Config::getDouble( "ObjectTracker", "CarGate", 0.5))
  , mPersonGate((float)oadrive::util::Config::getDouble( "ObjectTracker", "PersonGate", 0.5))
  , mCarGateForNewFilter((float)oadrive::util::Config::getDouble( "ObjectTracker", "CarGateForNewFilter", 20))
  , mPersonGateForNewFilter((float)oadrive::util::Config::getDouble( "ObjectTracker", "PersonGateForNewFilter", 20))
//  , mCarPose_odo(0,0,0)
  , mMaxTimeBeforeDeletionSign((float)oadrive::util::Config::getDouble( "ObjectTracker", "MaxTimeBeforeDeletionSign", 10))
  , mMaxTimeBeforeDeletionCar((float)oadrive::util::Config::getDouble( "ObjectTracker", "MaxTimeBeforeDeletionCar", 3))
  , mMaxTimeBeforeDeletionPerson((float)oadrive::util::Config::getDouble( "ObjectTracker", "MaxTimeBeforeDeletionPerson", 3))
  , mMaxTimeBeforeDeletionObstacle((float)oadrive::util::Config::getDouble( "ObjectTracker", "MaxTimeBeforeDeletionObstacle", 3))
  , mID_counter(0)  
{
  cv::Point2f topLeft((float)oadrive::util::Config::getDouble( "ObjectTracker", "ROI_xMax", 3),
                 (float)oadrive::util::Config::getDouble( "ObjectTracker", "ROI_yMax", 1.5));
  cv::Point2f bottomRight((float)oadrive::util::Config::getDouble( "ObjectTracker", "ROI_xMin", -1),
                 (float)oadrive::util::Config::getDouble( "ObjectTracker", "ROI_yMin", -1.5));
  mROI = cv::Rect(topLeft,bottomRight);
}

ObjectTracker::~ObjectTracker()
{

}

void ObjectTracker::trackObjects(const DetectedObjects &detectedObjects)
{
  if(!detectedObjects.cars.empty())
    this->trackCars(detectedObjects.cars);
  if(!detectedObjects.persons.empty())
    this->trackPersons(detectedObjects.persons);
}

void ObjectTracker::trackCars(std::vector<DetectedCar> detectedCars)
{
  if(!mTrackedCars.empty())
  {
    // data association
    std::vector<std::vector<float> > measurementMahanalobisDistances;
    for(DetectedCar &detectedCar : detectedCars)
    {
      std::vector<float> DistancesToTrackedCars;
      Eigen::Vector2f Z((float)detectedCar.object.pose.getX(),(float)detectedCar.object.pose.getY());
      Eigen::MatrixXf R(2,2);
      R << detectedCar.object.poseVariance.getX(), 0.0f, 0.0f, detectedCar.object.poseVariance.getY();
      for(CarFilter & carFilter : mTrackedCars)
      {
        // Gating
        float mahalanobisDistance = carFilter.poseFilter->getMahalanobisDistance(Z,R);
        if(mahalanobisDistance < mCarGate)
        {
          DistancesToTrackedCars.push_back(mahalanobisDistance);
        }
        else
        {
          DistancesToTrackedCars.push_back(FLT_MAX);
        }
      }
      measurementMahanalobisDistances.push_back(DistancesToTrackedCars);
    }
    std::vector<int> measurementID_forEachCar = this->nearestNeighborDataAssociation(measurementMahanalobisDistances);

    // update filters
    this->updateCars(detectedCars, measurementID_forEachCar);

    // create new filters if far away from gate/filterPose
    for(unsigned int detectedCarID = 0; detectedCarID < measurementMahanalobisDistances.size(); detectedCarID++)
    {
      // check wether the distances between detectedCar and all existing filters are greater than <mCarGateForNewFilter>.
      // If so: Create new filter for detected Car
      unsigned int outOfGateCounter = 0;
      for(float &distanceToFilter : measurementMahanalobisDistances[detectedCarID])
      {
        if(distanceToFilter > mCarGateForNewFilter)
        {
          outOfGateCounter++;
        }
      }
      if(outOfGateCounter == measurementMahanalobisDistances[detectedCarID].size())
      {
        mTrackedCars.push_back(this->createCarFilter(detectedCars[detectedCarID]));
      }
    }
  }
  // if not a single object is tracked already, filters are created for each measurement
  else
  {
    for(DetectedCar & detectedCar : detectedCars)
    {
      mTrackedCars.push_back(this->createCarFilter(detectedCar));
    }
  }
}

void ObjectTracker::trackPersons(std::vector<DetectedPerson> detectedPersons)
{
  if(!mTrackedPersons.empty())
  {
    // data association
    std::vector<std::vector<float> > measurementMahanalobisDistances;
    for(DetectedPerson &detectedPerson : detectedPersons)
    {
      if (detectedPerson.height > 0.7) {
        // std::cout << "yes! " << mTrackedPersons.size() << std::endl;
      }
      std::vector<float> DistancesToTrackedPersons;
      Eigen::Vector2f Z((float)detectedPerson.object.pose.getX(),(float)detectedPerson.object.pose.getY());
      Eigen::MatrixXf R(2,2);
      R << detectedPerson.object.poseVariance.getX(), 0.0f, 0.0f, detectedPerson.object.poseVariance.getY();
      for(PersonFilter & personFilter : mTrackedPersons)
      {
        // Gating
        float mahalanobisDistance = personFilter.poseFilter->getMahalanobisDistance(Z,R);
        if(mahalanobisDistance < mPersonGate)
        {
          DistancesToTrackedPersons.push_back(mahalanobisDistance);
        }
        else
        {
          DistancesToTrackedPersons.push_back(FLT_MAX);
        }
      }
      measurementMahanalobisDistances.push_back(DistancesToTrackedPersons);
    }
    std::vector<int> measurementID_forEachPerson = this->nearestNeighborDataAssociation(measurementMahanalobisDistances);

    // update filters
    this->updatePersons(detectedPersons, measurementID_forEachPerson);

    // create new filters if far away from gate/filterPose
    for(unsigned int detectedPersonID = 0; detectedPersonID < measurementMahanalobisDistances.size(); detectedPersonID++)
    {
      // check wether the distances between detectedCar and all existing filters are greater than <mCarGateForNewFilter>.
      // If so: Create new filter for detected Car
      unsigned int outOfGateCounter = 0;
      for(float &distanceToFilter : measurementMahanalobisDistances[detectedPersonID])
      {
        // std::cout << detectedPersonID << " " << distanceToFilter << std::endl;
        if(distanceToFilter > mPersonGateForNewFilter)
        {
          outOfGateCounter++;
        }
      }
      if(outOfGateCounter == measurementMahanalobisDistances[detectedPersonID].size())
      {
        mTrackedPersons.push_back(this->createPersonFilter(detectedPersons[detectedPersonID]));
      }
    }
  }
  // if not a single object is tracked already, filters are created for each measurement
  else
  {
    for(DetectedPerson & detectedPerson : detectedPersons)
    {
      mTrackedPersons.push_back(this->createPersonFilter(detectedPerson));
    }
  }
}

std::vector<int> ObjectTracker::nearestNeighborDataAssociation(std::vector<std::vector< float > > mahalanobisDistances)
{
  std::vector<int> objectIDs;
  //associate each measurement to nearest tracked Object. "-1" if no measurement passed the gating
  for(std::vector<float> &measurement : mahalanobisDistances)
  {
    int objectID = std::distance(measurement.begin(),std::min_element(measurement.begin(),measurement.end()));
    if(measurement[objectID] == FLT_MAX)
      objectID = -1;
    objectIDs.push_back(objectID);
  }

  // each tracked object can only be associated with one measurement i.e. the nearest
  std::vector<int> measurementID_ofObjects(mahalanobisDistances[0].size(),-1);
  for(unsigned int measurementID = 0; measurementID < objectIDs.size(); measurementID++)
  {
    // check if measurement has an object
    if(objectIDs[measurementID] != -1)
    {
      // check if object filter already has an associated measurement
      if(measurementID_ofObjects[objectIDs[measurementID]] == -1)
      {
        measurementID_ofObjects[objectIDs[measurementID]] = measurementID;
      }
      // check if new measurement is nearer than the current nearest measurement
      else if(mahalanobisDistances.at(measurementID).at(objectIDs[measurementID])
              < mahalanobisDistances.at(measurementID_ofObjects[objectIDs[measurementID]]).at(objectIDs[measurementID]))
      {
        measurementID_ofObjects[objectIDs[measurementID]] = measurementID;
      }
    }
  }
  return measurementID_ofObjects;
}

void ObjectTracker::updateCars(std::vector<DetectedCar> detectedCars, std::vector<int> measurementID_forEachCar)
{
  for(size_t filterID = 0; filterID < mTrackedCars.size(); filterID++)
  {
    int measurementID = measurementID_forEachCar[filterID];
    if(measurementID != -1)
    {
      Eigen::Vector2f Z_pose((float)detectedCars[measurementID].object.pose.getX(),(float)detectedCars[measurementID].object.pose.getY());
      Eigen::MatrixXf R_pose(2,2);
      R_pose << detectedCars[measurementID].object.poseVariance.getX(), 0.0f, 0.0f, detectedCars[measurementID].object.poseVariance.getY();
      mTrackedCars[filterID].poseFilter->update(Z_pose,R_pose);
      // std::cout << "ErrorVariance: " << mTrackedCars[filterID].poseFilter->getErrorCovariance() << std::endl;

      Eigen::VectorXf Z_width(1), R_width(1);
      Z_width << detectedCars[measurementID].object.width;
      R_width << detectedCars[measurementID].object.widthVariance;
      mTrackedCars[filterID].widthFilter->update(Z_width, R_width);
      mTrackedCars[filterID].lastMeasurementTime = time(NULL);
      mTrackedCars[filterID].numberOfMeasurements++;
    }
    else
    {
      mTrackedCars[filterID].poseFilter->update();
      mTrackedCars[filterID].widthFilter->update();
    }
  }
}

void ObjectTracker::updatePersons(std::vector<DetectedPerson> detectedPersons, std::vector<int> measurementID_forEachPerson)
{
  for(size_t filterID = 0; filterID < mTrackedPersons.size(); filterID++)
  {
    int measurementID = measurementID_forEachPerson[filterID];
    if(measurementID != -1)
    {
      Eigen::Vector2f Z_pose((float)detectedPersons[measurementID].object.pose.getX(),(float)detectedPersons[measurementID].object.pose.getY());
      Eigen::MatrixXf R_pose(2,2);
      R_pose << detectedPersons[measurementID].object.poseVariance.getX(), 0.0f, 0.0f, detectedPersons[measurementID].object.poseVariance.getY();
      mTrackedPersons[filterID].poseFilter->update(Z_pose,R_pose);
      // std::cout << "ErrorVariance: " << mTrackedCars[filterID].poseFilter->getErrorCovariance() << std::endl;

      Eigen::VectorXf Z_width(1), R_width(1);
      Z_width << detectedPersons[measurementID].object.width;
      R_width << detectedPersons[measurementID].object.widthVariance;
      mTrackedPersons[filterID].widthFilter->update(Z_width, R_width);

      Eigen::VectorXf Z_height(1), R_height(1);
      Z_height << detectedPersons[measurementID].height;
      R_height << detectedPersons[measurementID].heightVariance;
      mTrackedPersons[filterID].heightFilter->update(Z_height, R_height);

      mTrackedPersons[filterID].lastMeasurementTime = time(NULL);
      mTrackedPersons[filterID].numberOfMeasurements++;
    }
    else
    {
      mTrackedPersons[filterID].poseFilter->update();
      mTrackedPersons[filterID].widthFilter->update();
    }
  }
}

TrackedObjects ObjectTracker::getTrackedObjects(const ExtendedPose2d &carPose_odo)
{
  TrackedObjects objects;
  this->checkAndDeleteObjects(carPose_odo);
  objects.cars = this->getTrackedCars();
  objects.persons = this->getTrackedPersons();
  objects.follow_car_id = this->getCarFollowingId(carPose_odo, objects.cars);
  objects.obstacles = this->getTrackedObstacles();
  objects.trafficSigns = this->getTrackedTrafficSigns();
  return objects;
}

std::vector<TrackedCar> ObjectTracker::getTrackedCars()
{
  // getFilters
  std::vector<TrackedCar> trackedCars;
  for(CarFilter & carFilter : mTrackedCars)
  {
    carFilter.poseFilter->update();
//    carFilter.widthFilter->update();
    TrackedCar trackedCar;
    VectorXf state_pose = carFilter.poseFilter->getState();
    trackedCar.object.pose.setX(state_pose(0));
    trackedCar.object.pose.setY(state_pose(1));
    trackedCar.object.poseErrorCovariance = carFilter.poseFilter->getErrorCovariance();
    VectorXf state_width = carFilter.widthFilter->getState();
    trackedCar.object.width = state_width(0);
    MatrixXf widthErrorCovariance = carFilter.widthFilter->getErrorCovariance();
    trackedCar.object.widthErrorCovariance = widthErrorCovariance(0);
    trackedCar.object.filterID = carFilter.filterID;
    trackedCar.speed.setX(state_pose(2));
    trackedCar.speed.setY(state_pose(3));
    trackedCar.object.numberOfMeasurements = carFilter.numberOfMeasurements;
    trackedCars.push_back(trackedCar);
  }
  return trackedCars;
}

std::vector<TrackedPerson> ObjectTracker::getTrackedPersons()
{
  // getFilters
  std::vector<TrackedPerson> trackedPersons;
  for(PersonFilter & personFilter : mTrackedPersons)
  {
    personFilter.poseFilter->update();
//    carFilter.widthFilter->update();
    TrackedPerson trackedPerson;
    VectorXf state_pose = personFilter.poseFilter->getState();
    trackedPerson.object.pose.setX(state_pose(0));
    trackedPerson.object.pose.setY(state_pose(1));
    trackedPerson.object.poseErrorCovariance = personFilter.poseFilter->getErrorCovariance();
    VectorXf state_width = personFilter.widthFilter->getState();
    trackedPerson.object.width = state_width(0);
    MatrixXf widthErrorCovariance = personFilter.widthFilter->getErrorCovariance();
    trackedPerson.object.widthErrorCovariance = widthErrorCovariance(0);
    trackedPerson.object.filterID = personFilter.filterID;
    trackedPerson.object.numberOfMeasurements = personFilter.numberOfMeasurements;
    VectorXf state_height = personFilter.heightFilter->getState();
    trackedPerson.height = state_height(0);
    MatrixXf heightErrorCovariance = personFilter.heightFilter->getErrorCovariance();
    trackedPerson.heightErrorCovariance = heightErrorCovariance(0);
    trackedPersons.push_back(trackedPerson);
  }
  return trackedPersons;
}

std::vector<TrackedObject> ObjectTracker::getTrackedObstacles()
{
  std::vector<TrackedObject> trackedObstacles;
  for(ObstacleFilter & obstacleFilter : mTrackedObstacles)
  {
    TrackedObject obstacle;
    obstacle.pose = obstacleFilter.pose;
    obstacle.filterID = obstacleFilter.filterID;
    trackedObstacles.push_back(obstacle);
  }
  return trackedObstacles;
}

std::vector<TrafficSign> ObjectTracker::getTrackedTrafficSigns()
{
  std::vector<TrafficSign> trackedTrafficSigns;
  for(TrafficSignFilter & trafficSignFilter : mTrackedTrafficSigns)
  {
    trackedTrafficSigns.push_back(trafficSignFilter.trafficSign);
  }
  return trackedTrafficSigns;
}

void ObjectTracker::checkAndDeleteObjects(const ExtendedPose2d &carPose_odo){

  //cars: delete after X seconds or if outside roi
  unsigned int filterID = 0; // using int instead of iterators to acces different/multiple vectors
  while(filterID < mTrackedCars.size())
  {
    VectorXf state = mTrackedCars[filterID].poseFilter->getState();
    ExtendedPose2d pose_odo(state(0),state(1),0);
    ExtendedPose2d pose_ego = mCoordinateConverter->world2Car(carPose_odo,pose_odo);
    cv::Point2f pose(pose_ego.getX(),pose_ego.getY());
    if(!mROI.contains(pose))
    {
      LOGGING_INFO(obstacleLogger, "[obstacleLogger] Object outside ROI! Deleted! Pos: " << pose_ego.getX() << " " << pose_ego.getY() << endl);
      mTrackedCars.erase(mTrackedCars.begin()+filterID);
    }
    else if((float)( time(NULL) - mTrackedCars[filterID].lastMeasurementTime ) > mMaxTimeBeforeDeletionCar)
    {
      LOGGING_INFO(obstacleLogger, "[obstacleLogger] Object not detected for " << mMaxTimeBeforeDeletionCar << "! Deleted!" << endl);
      mTrackedCars.erase(mTrackedCars.begin()+filterID);
    }
    else
    {
      filterID++;
    }
  }

  // Persons
  filterID = 0; // using int instead of iterators to acces different/multiple vectors
  while(filterID < mTrackedPersons.size())
  {
    VectorXf state = mTrackedPersons[filterID].poseFilter->getState();
    ExtendedPose2d pose_odo(state(0),state(1),0);
    ExtendedPose2d pose_ego = mCoordinateConverter->world2Car(carPose_odo,pose_odo);
    cv::Point2f pose(pose_ego.getX(),pose_ego.getY());
    if(!mROI.contains(pose))
    {
      LOGGING_INFO(obstacleLogger, "[obstacleLogger] Object outside ROI! Deleted! Pos: " << pose_ego.getX() << " " << pose_ego.getY() << endl);
      mTrackedPersons.erase(mTrackedPersons.begin()+filterID);
    }
    else if((float)( time(NULL) - mTrackedPersons[filterID].lastMeasurementTime ) > mMaxTimeBeforeDeletionPerson)
    {
      LOGGING_INFO(obstacleLogger, "[obstacleLogger] Object not detected for " << mMaxTimeBeforeDeletionPerson << "! Deleted!" << endl);
      mTrackedPersons.erase(mTrackedPersons.begin()+filterID);
    }
    else
    {
      filterID++;
    }
  }

  //obstacles: delete after X seconds
  filterID = 0;
  while(filterID < mTrackedObstacles.size())
  {
    if((float)( time(NULL) - mTrackedObstacles[filterID].lastMeasurementTime ) > mMaxTimeBeforeDeletionObstacle)
    {
      LOGGING_INFO(obstacleLogger, "[obstacleLogger] Obstacle not detected for " << mMaxTimeBeforeDeletionObstacle << "! Deleted!" << endl);
      mTrackedObstacles.erase(mTrackedObstacles.begin() + filterID);
    }
    else
    {
      filterID++;
    }
  }

  //trafficSigns: delete if outside ROI
  filterID = 0;
  while(filterID < mTrackedTrafficSigns.size())
  {
    ExtendedPose2d pose_odo = mTrackedTrafficSigns[filterID].trafficSign.getPose();

    ExtendedPose2d pose_ego = mCoordinateConverter->world2Car(carPose_odo, pose_odo);
    if(!mROI.contains(cv::Point2f(pose_ego.getX(), pose_ego.getY())))
    {
      LOGGING_INFO(obstacleLogger, "[obstacleLogger] Traffic sign outside ROI! Deleted!" << pose_ego.getX() << " " << pose_ego.getY() << endl);
      mTrackedTrafficSigns.erase(mTrackedTrafficSigns.begin()+filterID);
    }
    else if((float)( time(NULL) - mTrackedTrafficSigns[filterID].lastMeasurementTime ) > mMaxTimeBeforeDeletionSign)
    {
      LOGGING_INFO(obstacleLogger, "[obstacleLogger] Traffic sign not detected for " << mMaxTimeBeforeDeletionSign << "! Deleted!" << endl);
      mTrackedTrafficSigns.erase(mTrackedTrafficSigns.begin()+filterID);
    }
    else
    {
      filterID++;
    }
  }
}

CarFilter ObjectTracker::createCarFilter(DetectedCar detectedCar)
{
  LOGGING_INFO(obstacleLogger, "[obstacleLogger] Creating new car!" << endl);
  std::shared_ptr<SystemModel> constVelocityModel(new ConstVelocity(mCarSpectralPowerDensity));
  std::shared_ptr<KalmanFilter> poseFilter(new KalmanFilter(constVelocityModel));
  Eigen::Vector4f x_pose((float)detectedCar.object.pose.getX(),(float)detectedCar.object.pose.getY(),0.0f,0.0f);
  Eigen::Vector4f P_vec(detectedCar.object.poseVariance.getX(), detectedCar.object.poseVariance.getY(),
                        2*detectedCar.object.poseVariance.getX()*15*15, 2*detectedCar.object.poseVariance.getY()*15*15);
  Eigen::MatrixXf P_pose = P_vec.asDiagonal();
  poseFilter->init(x_pose,P_pose);

  std::shared_ptr<SystemModel> constValueModel(new ConstValue(mCarSpectralPowerDensity));
  std::shared_ptr<KalmanFilter> widthFilter(new KalmanFilter(constValueModel));
  Eigen::VectorXf x_width(1), P_width(1);
  x_width << detectedCar.object.width;
  P_width << detectedCar.object.widthVariance;
  widthFilter->init(x_width, P_width);

  CarFilter carFilter;
  carFilter.poseFilter = poseFilter;
  carFilter.widthFilter = widthFilter;
  carFilter.filterID = mID_counter++;
  carFilter.lastMeasurementTime = time(NULL);
  carFilter.numberOfMeasurements = 1;
  return carFilter;
}

PersonFilter ObjectTracker::createPersonFilter(DetectedPerson detectedPerson)
{
  LOGGING_INFO(obstacleLogger, "[obstacleLogger] Creating new person!" << endl);
  
  /*std::shared_ptr<SystemModel> constVelocityModel(new ConstVelocity(mPersonSpectralPowerDensity));
  std::shared_ptr<KalmanFilter> poseFilter(new KalmanFilter(constVelocityModel));
  Eigen::Vector4f x_pose((float)detectedPerson.object.pose.getX(),(float)detectedPerson.object.pose.getY(),0.0f,0.0f);
  Eigen::Vector4f P_vec(detectedPerson.object.poseVariance.getX(), detectedPerson.object.poseVariance.getY(),
                        2*detectedPerson.object.poseVariance.getX()*15*15, 2*detectedPerson.object.poseVariance.getY()*15*15);
  Eigen::MatrixXf P_pose = P_vec.asDiagonal();
  poseFilter->init(x_pose,P_pose);*/

  /*std::shared_ptr<SystemModel> constValueModel_2(new ConstValue4d(mPersonSpectralPowerDensity));
  std::shared_ptr<KalmanFilter> poseFilter(new KalmanFilter(constValueModel_2));
  Eigen::Vector4f x_pose((float)detectedPerson.object.pose.getX(),(float)detectedPerson.object.pose.getY(),0.0f,0.0f);
  Eigen::Vector4f P_vec(detectedPerson.object.poseVariance.getX(), detectedPerson.object.poseVariance.getY(),
                        detectedPerson.object.poseVariance.getX(), detectedPerson.object.poseVariance.getY());
  Eigen::MatrixXf P_pose = P_vec.asDiagonal();
  poseFilter->init(x_pose, P_pose);*/

  std::shared_ptr<SystemModel> constValueModel_2(new ConstValue2d(mPersonSpectralPowerDensity));
  std::shared_ptr<KalmanFilter> poseFilter(new KalmanFilter(constValueModel_2));
  Eigen::Vector2f x_pose((float)detectedPerson.object.pose.getX(),(float)detectedPerson.object.pose.getY());
  Eigen::Vector2f P_vec(detectedPerson.object.poseVariance.getX(), detectedPerson.object.poseVariance.getY());
  Eigen::MatrixXf P_pose = P_vec.asDiagonal();
  poseFilter->init(x_pose, P_pose);

  // // LOGGING_INFO(obstacleLogger, "[obstacleLogger] Creating new car!" << endl);
  // std::shared_ptr<SystemModel> constVelocityModel(new ConstVelocity(mCarSpectralPowerDensity));
  // std::shared_ptr<KalmanFilter> poseFilter(new KalmanFilter(constVelocityModel));
  // Eigen::Vector4f x_pose((float)detectedPerson.object.pose.getX(),(float)detectedPerson.object.pose.getY(),0.0f,0.0f);
  // Eigen::Vector4f P_vec(detectedPerson.object.poseVariance.getX(), detectedPerson.object.poseVariance.getY(),
  //                       2*detectedPerson.object.poseVariance.getX()*15*15, 2*detectedPerson.object.poseVariance.getY()*15*15);
  // Eigen::MatrixXf P_pose = P_vec.asDiagonal();
  // poseFilter->init(x_pose,P_pose);

  std::shared_ptr<SystemModel> constValueModel(new ConstValue(mPersonSpectralPowerDensity));
  std::shared_ptr<KalmanFilter> widthFilter(new KalmanFilter(constValueModel));
  Eigen::VectorXf x_width(1), P_width(1);
  x_width << detectedPerson.object.width;
  P_width << detectedPerson.object.widthVariance;
  widthFilter->init(x_width, P_width);

  std::shared_ptr<SystemModel> constValueModel_3(new ConstValue(mPersonSpectralPowerDensity));
  std::shared_ptr<KalmanFilter> heightFilter(new KalmanFilter(constValueModel_3));
  Eigen::VectorXf x_height(1), P_height(1);
  x_height << detectedPerson.height;
  P_height << detectedPerson.heightVariance;
  heightFilter->init(x_height, P_height);

  PersonFilter personFilter;
  personFilter.poseFilter = poseFilter;
  personFilter.widthFilter = widthFilter;
  personFilter.heightFilter = heightFilter;
  personFilter.filterID = mID_counter++;
  personFilter.lastMeasurementTime = time(NULL);
  personFilter.numberOfMeasurements = 1;
  return  personFilter;
}

void ObjectTracker::trackUS_obstacle(std::vector<ExtendedPose2d> US_position)
{
  if (US_position.empty()) {
    return;
  }
  //!!! currently only one obstacle can be tracked
  // average pose
  ExtendedPose2d averagedUS_pose = this->averagePoses(US_position);

  if(mTrackedObstacles.empty())
  {
    // create "filter"
    ObstacleFilter obstacle;
    obstacle.pose = averagedUS_pose;
    obstacle.poseList.push_back(averagedUS_pose);
    obstacle.filterID = mID_counter++;
    obstacle.lastMeasurementTime = time(NULL);
    mTrackedObstacles.push_back(obstacle);
    LOGGING_INFO(obstacleLogger, "[obstacleLogger] Create obstacle!" << endl);
  }
  else
  {
    // update "filter"
    if(mTrackedObstacles.back().poseList.size() == 5)
      mTrackedObstacles.back().poseList.erase(mTrackedObstacles.back().poseList.begin());
    mTrackedObstacles.back().poseList.push_back(averagedUS_pose);
    mTrackedObstacles.back().lastMeasurementTime = time(NULL);
    mTrackedObstacles.back().pose = this->averagePoses(mTrackedObstacles.back().poseList);
  }
}

void ObjectTracker::trackTrafficSign(oadrive::world::TrafficSign trafficSign)
{
  // Warning: no real data association method implemented for traffic signs. First already tracked traffic sign will get the new measurement;
  bool isSignAlreadyTracked = false;
  for(TrafficSignFilter & trafficSignFilter : mTrackedTrafficSigns)
  {
    ExtendedPose2d trafficSignPose = trafficSign.getPose();
    if( trafficSignFilter.trafficSign.calcDistTo(trafficSignPose) < 0.4f && trafficSignFilter.trafficSign.getType() == trafficSign.getType())
    {
      // update measurement!
      isSignAlreadyTracked = true;
      if(trafficSignFilter.poseList.size() == 5)
        trafficSignFilter.poseList.erase(trafficSignFilter.poseList.begin());
      trafficSignFilter.poseList.push_back(trafficSignPose);
      trafficSignFilter.trafficSign.setPose(this->averagePoses(trafficSignFilter.poseList));
      trafficSignFilter.lastMeasurementTime = time(NULL);
    }
  }
  if(!isSignAlreadyTracked)
  {
    // create new filter
    LOGGING_INFO(obstacleLogger, "[obstacleLogger] Detected new traffic sign!" << endl);
    TrafficSignFilter trafficSignFilter(trafficSign);
    trafficSignFilter.poseList.push_back(trafficSign.getPose());
    trafficSignFilter.trafficSign.setId(mID_counter++);
    trafficSignFilter.lastMeasurementTime = time(NULL);
    mTrackedTrafficSigns.push_back(trafficSignFilter);
  }
}

ExtendedPose2d ObjectTracker::averagePoses(std::vector<ExtendedPose2d> poseList)
{
  float x = 0;
  float y = 0;
  for(ExtendedPose2d & poses : poseList)
  {
    x += poses.getX();
    y += poses.getY();
  }
  return ExtendedPose2d(x/poseList.size(), y/poseList.size(), poseList.back().getYaw());
}

int ObjectTracker::getCarFollowingId(const ExtendedPose2d &carPose_odo, const std::vector<TrackedCar> &trackedCars)
{
  int carFollowingId = -1;
  for(TrackedCar car : trackedCars)
  {
    ExtendedPose2d trackedCar_ego = mCoordinateConverter->world2Car(carPose_odo, car.object.pose);
    float distanceToTrackedCar = sqrt(trackedCar_ego.getX()*trackedCar_ego.getX()
                                      + trackedCar_ego.getY()*trackedCar_ego.getY());

    float totalSpeed = sqrt(car.speed.getY() * car.speed.getY() + car.speed.getX() * car.speed.getX());
    // Only follow driving cars
    if (totalSpeed < 0.05) {
      continue;
    }

    float angleDifference = atan2(car.speed.getY(), car.speed.getX()) - carPose_odo.getYaw();
    if(angleDifference > M_PI)
      angleDifference -= (2*M_PI);
    else if(angleDifference < -M_PI)
      angleDifference += (2*M_PI);
    
    if(trackedCar_ego.getX() > 0.8f && distanceToTrackedCar < 3.0f && fabs(angleDifference) < M_PI/2)
    {
      carFollowingId = car.object.filterID;
    }
  }


  return carFollowingId;
}

void ObjectTracker::reset()
{
  mTrackedCars.clear();
  mTrackedPersons.clear();
  mTrackedObstacles.clear();
  mTrackedTrafficSigns.clear();
}

} // namespace
}

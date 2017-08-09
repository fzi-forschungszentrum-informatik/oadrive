// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2017 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-16
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_OBSTACLE_H
#define OADRIVE_WORLD_OBSTACLE_H

#include <boost/shared_ptr.hpp>
#include <list>

#include "EnvObject.h"
#include "EventRegion.h"
#include "oadrive_core/ExtendedPose2d.h"
#include "opencv2/video/tracking.hpp"

#define EXISTENCE_THRES 0.75
#define OBSTACLE_REGION_BORDER 3.0
#define OBSTACLE_REGION_BORDER_SMALL 2.0
#define OBSTACLE_REGION_CROSSED 0.3
#define TIME_TO_LIVE_US 20
#define TIME_TO_LIVE_DEPTH 100

using namespace oadrive::core;


namespace oadrive{
namespace world{

class Obstacle;	// predefine so shared_ptr can use it
typedef boost::shared_ptr<Obstacle> ObstaclePtr;		// define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<Obstacle> > ObstaclePtrList;    
enum SensorType{
  US,
  DEPTH,
  TRAFFICSIGN
};

class Obstacle: public EnvObject
{
public:
  Obstacle(const ExtendedPose2d &pose, double width, double height, SensorType type);
  //!
  //! \brief update updates the position and teh probaility of the object with mor or less good algorithmens
  //! \param pos
  //! \param mType
  //!
  void update(const ExtendedPose2d &pos, SensorType mType);

  //! get the maximum speed regarding to the distance to this object
  double getMaxSpeed(double distance);

  //! set new pose and update event regions
  void setPose(const oadrive::core::ExtendedPose2d &mPose);
  //! get time to live (in seconds)
  int getTimeToLive();
  //! set time to live in seconds. ( will be checked by the enviromend in remove old obstacles
  void setTimeToLive(int TTL);
  //!
  //! \brief addEventRegion add a Eventregion to the object
  //! \param evRegion
  //! \note you also have to store the event region in the enviroment.
  void addEventRegion( EventRegionPtr evRegion );
  //! get how often this object was saw by ultrasonic sensor
  int getSawUS();
  //! get how often this object was saw by the depth camera
  int getSawDepth();
  //! if object is a traffic sign
  int getSawTrafficSign();
  //! get if is relevant obstacle for ACC only for debug
  bool getIsRelevantAccDebugOnly() const;
  //! set is relevant obstacle (is done by the Enviroment getMaxSpeed
  void setIsRelevantAcc(bool value);
  //! get a list of all Eventregions related to this object (note the eventregions also saved in the enviroment)
  EventRegionPtrList *getEventRegions();
  //! \brief setFree reduce the probaility depending on the sensor type
  //! \param type sensor type
  void setFree(SensorType type);

  bool isDoNotDelete();
  void setDoNotDelete(bool del);

  SensorType getSensorType() { return mSensorType; }

  bool isStatic() { return mStatic; }
  void setStatic( bool isStatic ) { mStatic = isStatic; }

private:
  //! object was saw from the ultrasonic sensor n Timer
  int mSawUS;
  //! object was saw from the Depth camera n Times
  int mSawDepth;
  //! this object is a traffic sign and will not be moved
  int mSawTrafficSign;
  //! Time to live in seconds
  int mTTL;
  //! for debug only. Mark if obstacle is relevant for ACC
  bool mIsRelevantAcc;
  //! Kalman filter (just for the postion so it is more a least square estimator but it would be easier to extend it)
  cv::KalmanFilter mKalmanFilter;
  //! list to all eventregions related to the obstacle
  EventRegionPtrList mEventRegions;
  //!should not be deleted
  bool mDoNotDelete;

  SensorType mSensorType;

  bool mStatic;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}//namespace
}//namespace

#endif // OADRIVE_WORLD_OBSTACLE_H

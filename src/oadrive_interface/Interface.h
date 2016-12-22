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
 * \date    2015-12-01
 *
 * Interface to Simulation and ADTF.
 * All communication with the outside world should go through this interface.
 *
 */
//----------------------------------------------------------------------


#ifndef OADRIVE_INTERFACE_INTERFACE_H
#define OADRIVE_INTERFACE_INTERFACE_H
//activate this if you can't use imgwrite/read with PNG
//#define noPng
#include <boost/thread/mutex.hpp>
#include <oadrive_util/BirdViewConverter.h>
#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_util/InitLogger.h>
#include <oadrive_lanedetection/StreetPatcher.h>
#include <oadrive_world/Environment.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <oadrive_missioncontrol/MissionControl.h>
#include <oadrive_missioncontrol/IControl4MC.h>		// Event receiver for MissionControl
#include <oadrive_control/DriverModule.h>
#include <oadrive_util/Timer.h>
#include <oadrive_trafficsign/TrafficSignDetAruco.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <oadrive_interface/interfaceLogging.h>
#include <oadrive_obstacle/ProcessSensors.h>
#include <opencv2/flann/timer.h>

namespace oadrive{
namespace interface{

struct JuryCommand
{
  juryActions action;
  int maneuverEntryID;
};

/*! Interface between oadrive logic and car.
         * Can be used as an interface to a simulation, or ADTF Filters.
         * Abstracts away all other modules, such as lane-detection and decision-making.
         * Minimum input is an OpenCV camera image and a car pose. */
class Interface : public oadrive::missioncontrol::IControl4MC,
  public oadrive::util::TimerEventListener
{
public:
  /*! Constructor
   * Needs to be given a file name to where the config file is.
   * \param iControl4MC (Optional) Event handler which will be called when
   * 		the mission control has new events (like communication with the jury).
   * 		If none is given, then the Interface class is responsible for handling
   * 		events by itself. */
  Interface( std::string configFolder, std::string carName, IControl4MC* iControl4MC = NULL );
  virtual ~Interface();

  /*! Pass the camera image to the class.
   * \param image RGB image, can be any pixel size (for example 640x480 or 1280x1024 pixel).
   * \param isBirdView set to true if the image is already conver in birdview, otherwise
   *            it will be converted to bird view using the parameters in the config file.*/
  void setCameraImage( cv::Mat image, bool isBirdview );

  /*! \brief setDepthImage pass the depth Image to the class. It is a 16bit open cv Mat. The resolution shoul be 320*240 otherwise you have to change CoordinateConverter
             *  \param image Depth Image */
  void setDepthImage(cv::Mat image);

  /*! Pass the current care position and orientation.
   * \param pose the car pose. X and Y are in meters, Yaw is in radians.
   * \note Yaw angle should be mathematically positive (increase counter-clockwise).
   * \note Yaw angle of 0 means the car is looking along the positive x-axis!
   * \note Until the first car pose is passed, the class assumes that the car is
   * 		standing at 0,0 and looking into direction 0 (positive x-axis).*/
  void setCarPose( const oadrive::core::ExtendedPose2d &pose );
  /*!
   * \brief setUsSensor Pass the data of the US-Sensor
   * \param distance all sensor distances must be in m
   */
  void setUsSensor(oadrive::obstacle::usSensor distance);

  /*! Returns current steering angle.
   * Returns mathematically positive steering/servo angle in radians.
   * 0 means "straight ahead"
   * Angle larger than 0 is "left", angle lower than zero means "right"
   * (TODO: CHECK THIS!)
   * angle range: -PI/6 . +PI/6. */
  float getSteering();

  /*! Returns speed in meters per second. */
  float getSpeed();

  /*! Lets the Interface know which action the jury wants us to perform.
   * \param action: (Quoting AADC Manual):
   *			- action_isReady (==0): The vehicle is asked if it is ready to start the
   *				maneuver given in maneuverEntry.
   *			- action_start (== 1): After receiving this sample the car should start the
   *				maneuver given in i16ManeuverEntry. Before that the sample the same
   *				maneuver is asked if it is ready.
   *			- action_stop (== -1): After receiving this sample the car should stop the
   *				maneuver given in i16ManeuverEntry. If the car is in a different
   *				maneuver it	should be stopped as well.
   * \param maneuverEntryID which maneuver in the maneuverlist should be performed
   * \note The commands are buffered and processed with the next setCameraImage command,
   *    for thread safety.
   * \see setManeuverList */
  void setJuryCommand( juryActions action, int maneuverEntryID );

  /*! Set the full maneuver list. Will be passed on to MissionControl.
   * The maneuver list is in XML format as described by the AADC Manual.
   * At this point, it has not been parsed - it is sent on as a full, raw string.
   * \note The list is buffered and processed with the next setCameraImage command,
   *    for thread safety. */
  void setManeuverList( std::string list );

  /*! Mission control Event: jury response
   * \note Do not call directly, will be called by MissionControl!
   * \see MissionControl IControl4MC::setJuryState */
  void setJuryState( stateCar state, int manID );
  /*! \brief incrementTime Increments the internal Timer
   * \param milliSeconds Time, which is elapsed since last call */
  void incrementTime(int milliSeconds);

  /*! Returns a pointer to the used environment.
   * Useful for getting information about the trajectory, street, and objects around the
   * car. */
  oadrive::world::EnvironmentPtr getEnvironment() { return Environment::getInstance(); }

  oadrive::lanedetection::StreetPatcher* getStreetPatcher() { return &mStreetPatcher; }


  void startDebugDumping( std::string path );
  cv::Mat generateDebugFeatureImage();
  cv::Mat generateDebugMap( int width, int height, float mapSize );

  oadrive::lanedetection::StreetPatcher* getPatcher() { return &mStreetPatcher; }
  oadrive::world::TrajectoryFactory* getTrajectoryFactory() {
    return &mTrajectoryFactory;
  }

  oadrive::util::Timer* getTimer() {return &mTimer;}

  /*! Toggle lights.
   * Implements IControl4MC.*/
  void setLights( enumLight light, bool on );


  /*! Returns the last image which we received: */
  cv::Mat getLastBirdViewImage();

  //! get depth Image processor for debug porpuse
  oadrive::obstacle::ProcessDepth* getDepthImageProcessor(){return mProcessSensor.getDepthImageProcessor();}

  oadrive::util::CoordinateConverter* getCoordConverter() { return &mCoordConverter; }

  /*! Implement TimerEventListener: */
  void eventTimerFired( timerType type, unsigned long timerID );

  void reset();

private:

  /*! Process all previous jury commands.
   * Empties the queue of jury commands received. Should be called in setCameraImage.*/
  void processJuryCommands();

  /*! Process any recently received maneuver list.*/
  void processNewManeuverList();

  void dumpDebugData( const ExtendedPose2d &pose, cv::Mat image );

  std::string mConfigPath;
  std::string mCarName;
  std::string mBirdViewCalFile;

  //we need mLogger only once!!!!
  oadrive::util::InitLogger mLogger;

  /*! Helper class for coord conversion (world, car and bird-view coordinates). */
  oadrive::util::CoordinateConverter mCoordConverter;

  oadrive::util::BirdViewConverter mBirdViewConverter;

  oadrive::util::Timer mTimer;

  oadrive::control::DriverModule mDriver;

  oadrive::lanedetection::StreetPatcher mStreetPatcher;

  oadrive::world::TrajectoryFactory mTrajectoryFactory;

  oadrive::missioncontrol::IControl4MC* mIControl4MC;
  oadrive::missioncontrol::MissionControl mMissionControl;
  oadrive::obstacle::ProcessSensors mProcessSensor;

  oadrive::trafficsign::TrafficSignDetAruco mTrafficSignDetectorAruco;

  int mImageCounter;

  cv::Mat mLastBirdViewImage;
  cv::Mat mLastBirdViewImageGray;

  bool mDebugActive;
  std::string mDebugOutputPath;

  float mMaximumFramesPerSecond;

  boost::posix_time::ptime mLastFrameReceivedTime;

  oadrive::core::Trajectory2d mLogTrajectory;
  //get from http://stackoverflow.com/questions/8593608/how-can-i-copy-a-directory-using-boost-filesystem
  bool copyDir(std::string sourcestr,std::string destinationStr);
  //! indicates if the car is driving slower due obstacles
  bool mACCOn;

  cvflann::StartStopTimer frameTimer;

  std::vector<JuryCommand> mJuryCommandQueue;
  std::string mManeuverList;
  bool mNewManeuverlistReceived;

  boost::mutex mtx;
  boost::mutex mtxCarPose;

  // last speed used to determine if breaking lights need to be switched on
  float mLastSpeed;

  // true if brake lights on
  bool mBrakeLightsOn;

  // true if reverse lights on
  bool mReverseLightsOn;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
}

#endif

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

#include "Interface.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <boost/filesystem.hpp>
#include <boost/date_time/local_time/local_time.hpp>
#include <oadrive_world/TrajectoryDatabase.h>
#include <oadrive_util/Config.h>
#include "interfaceLogging.h"
using namespace oadrive::core;
using namespace oadrive::util;
using namespace oadrive::world;
using namespace oadrive::missioncontrol;
using namespace boost::posix_time;
using namespace boost::filesystem;
using icl_core::logging::endl;
using icl_core::logging::flush;
//using namespace icl_core::logging;


namespace oadrive{
namespace interface{

#define noPngDepth //if you remove this, you have also to change DepthImage

Interface::Interface( std::string configFolder, std::string carName, IControl4MC* iControl4MC )
//we need mLogger only once
  : mConfigPath( Config::setConfigPath( configFolder, carName ) )
  , mCarName( carName )
  , mBirdViewCalFile( ( path(configFolder) / carName / "BirdviewCal.yml").string() )
  , mLogger( (path(configFolder) / "log.xml").string() )
  , mCoordConverter( mBirdViewCalFile )
  , mTimer()
  , mDriver()
  , mStreetPatcher( 3.0, &mCoordConverter, true, true )
  , mTrajectoryFactory()
  , mIControl4MC( iControl4MC )
  , mMissionControl( this, &mDriver, &mTrajectoryFactory, &mStreetPatcher )
  #ifdef noPngDepth
  , mProcessSensor (( path(configFolder) / carName / "DepthRef.yml").string(),( path(configFolder) / carName / "calUs.yml").string(),&mCoordConverter)
  #else
  , mProcessSensor (( path(configFolder) / carName / "DepthRef.png").string(),( path(configFolder) / carName / "calUs.yml").string(),&mCoordConverter)
  #endif
  , mTrafficSignDetectorAruco( &mCoordConverter, configFolder )
  , mImageCounter( 0 )		// for debugging
  , mLastBirdViewImage( mCoordConverter.getImgSizeBirdView(), CV_8UC3 )
  , mLastBirdViewImageGray( mCoordConverter.getImgSizeBirdView(), CV_8UC1 )
  , mDebugActive( false )
  , mDebugOutputPath( "" )
  , mMaximumFramesPerSecond( 0 )
  , mLastFrameReceivedTime( microsec_clock::local_time() )
  , mNewManeuverlistReceived( false )
  , mLastSpeed ( 0 )
  , mBrakeLightsOn ( false )
  , mReverseLightsOn ( false )
{
  LOGGING_INFO( interfaceLogger, "Initialized with config file: " << configFolder <<
                endl << flush);

  Environment::init( &mCoordConverter, &mDriver, &mTimer );

  // Let factory load all trajectories:
  path trajectoryDirectory( configFolder );
  //trajectoryDirectory = trajectoryDirectory.parent_path();
  trajectoryDirectory += "/trajectories/";
  //mTrajectoryFactory.loadTrajectoryDatabase( trajectoryDirectory.string() );
  TrajectoryDatabase::load( trajectoryDirectory.string() );
  LOGGING_INFO(interfaceLogger, "I think my name is "<<carName<<endl);


  Environment::getInstance()->setEventListener( &mMissionControl );
  mBirdViewConverter.loadConfig( mBirdViewCalFile );
  //mTimer();

  // give traffic sing detector bird view converter that it can get the camera matrix and distortion coefficients
  mTrafficSignDetectorAruco.setCameraParameters(mBirdViewConverter);

  // DEBUG: drive fixed trajectory:
  //mTrajectoryFactory.setFixedTrajectory( MANEUVER_PULLOUT_LEFT );
  //mTrajectoryFactory.generateFromPatches();

  mTimer.setTimer( 1000, TIMER_TYPE_SEND_MAP );
  mTimer.addListener( this );
}

Interface::~Interface()
{
  if( mDebugActive && mLogTrajectory.size() > 0 )
  {
    std::string trajFileName = mDebugOutputPath;
    trajFileName.append( "/finalTrajectory.txt" );
    std::ofstream trajFile( trajFileName.c_str() );
    trajFile << mLogTrajectory;
    trajFile.close();
  }
}

void Interface::reset()
{
  LOGGING_INFO( interfaceLogger, "----------------------------------------------" << endl );
  LOGGING_INFO( interfaceLogger, "-- Resetting world" << endl );
  LOGGING_INFO( interfaceLogger, "----------------------------------------------" << endl );
  mTimer.clearAllTimers();
  //setNewOrigin();
  Environment::reset();
  mStreetPatcher.reset();
  mDriver.reset();
  mTrajectoryFactory.clearOldTraj();
  // Should also be called by setting the empty trajectory, but just in case:
  Environment::getInstance()->setEventListener( &mMissionControl );
  mTimer.setTimer( 1000, TIMER_TYPE_SEND_MAP );
}

void Interface::setCameraImage( cv::Mat image, bool isBirdView )
{

  LOGGING_ERROR(interfaceLogger, " Interface got image .... " <<endl);
  // Get grayscale image
  cv::Mat gray;
  cv::cvtColor(image, gray, CV_BGR2GRAY);

  //cvflann::StartStopTimer timer;
  //timer.start();

  if( isBirdView )
  {
    mLastBirdViewImageGray = gray;
    mLastBirdViewImage = image;
  } else {
    mLastBirdViewImageGray = mBirdViewConverter.transform( gray );
    cv::cvtColor(mLastBirdViewImageGray, mLastBirdViewImage, CV_GRAY2BGR);
  }

  ExtendedPose2d carPoseCopy = mDriver.getCarPose();
  LOGGING_INFO(interfaceLogger, " Updating car pose .... " <<endl);
  Environment::getInstance()->updateCarPose( carPoseCopy );


  LOGGING_INFO(interfaceLogger, " Checking end of Trajectory .... " <<endl);
  bool endOfTrajctoryEvent = mDriver.checkEndOfTrajectoryFlag();
  if( endOfTrajctoryEvent )
  {
    mMissionControl.eventTrajectoryEndReached();
    Environment::getInstance()->setEndOfTrajReached(true);
  }

  //timer.stop();
  //float timeBirdView = timer.value;

  //timer.reset();
  //if(mDebugActive)
  //{
  //}

  //timer.start();

  // Get grayscale image:
  //cv::cvtColor( mLastBirdViewImage, grayBirdView, CV_BGR2GRAY);


  bool streetPatcherWasInitialized = mStreetPatcher.getIsInitialized();

  // Let the StreetPatcher know how many frames per meter currently recording:
  //mStreetPatcher.setFramesPerMeter( 6.0/std::max(mDriver.getSpeed(),0.1) );

  // Let Street Patcher find patches.
  // After this operation, the patches will be in the Environment.
  LOGGING_INFO(interfaceLogger, " Give the street patcher a new image .... " <<endl);
  mStreetPatcher.setImage( mLastBirdViewImageGray, carPoseCopy );

  // Let trajectory factory generate new trajectoy if needed.
  // The trajectory will be saved in the environment.

  LOGGING_INFO(interfaceLogger, " Updating the trajectory .... " <<endl);
  mTrajectoryFactory.requestUpdate(true);

  //timer.stop();
  //float timeImage = timer.value;

  //timer.reset();
  //timer.start();

  //timer.stop();
  //float timeDebugDump = timer.value;

  //std::cout << "bird view time: "<<timeBirdView<<std::endl
  //  << "image time: "<<timeImage<<std::endl
  //  << "debug time: "<<timeDebugDump<<std::endl;

  //mMaximumFramesPerSecond = 1.0/(timeImage + timeDebugDump);


  // give image to traffic sign detector

  LOGGING_INFO(interfaceLogger, " Detecting Traffic signs .... " <<endl);
  mTrafficSignDetectorAruco.detectMarkers(gray);


  ptime now = microsec_clock::local_time();
  time_duration timeDiff = now - mLastFrameReceivedTime;
  mLastFrameReceivedTime = now;
  //the timer should be set in the Image thread to avoid threading problems
  //this cause that the timer is more inaccurate
  incrementTime(timeDiff.total_milliseconds());

  LOGGING_INFO(interfaceLogger, " Clear obstacles .... " <<endl);
  Environment::getInstance()->clearObstacles( US );

  // Trigger processing of US Sensor data:

  LOGGING_INFO(interfaceLogger, " process sensors .... " <<endl);
  mProcessSensor.processUsSensor();

  // If new maneuver list of jury commands were received, process them:
  LOGGING_INFO(interfaceLogger, " Jury commands and man list .... " <<endl);
  processJuryCommands();
  processNewManeuverList();

  if( streetPatcherWasInitialized == false )
    if( mStreetPatcher.getIsInitialized() == true )
      mMissionControl.eventStreetPatcherIsInitialized();

  //timer.reset();
  //timer.start();

  //timer.stop();
  //float timeTrafficSign = timer.value;
  //std::cout << "traffic sign time: "<<timeTrafficSign<<std::endl;

  LOGGING_INFO(interfaceLogger, " Do the rest .... " <<endl);
  if( Broker::isActive() )
  {
    if( Broker::getInstance()->getTurnCommandReceived() )
    {
      mMissionControl.eventUTurn();
    }
    enumSpeedCommand command = Broker::getInstance()->getLastSpeedCommand();
    if( command != SPEED_COMMAND_NONE )
    {
      mMissionControl.setBoostCommand( command );
    }
    bool stopped = Broker::getInstance()->getHasReceivedStopCommand();
    if( stopped )
    {
      mMissionControl.eventParcourFinished();
    }
  }

  LOGGING_INFO(interfaceLogger, " Duming data .... " <<endl);
  if( mDebugActive )
    dumpDebugData( carPoseCopy, image );

  LOGGING_ERROR(interfaceLogger, " Interface has processed image. " <<endl);

}

void Interface::setDepthImage(cv::Mat image)
{

  LOGGING_INFO(interfaceLogger, " Entering Depth image processing ... " <<endl);
  LOGGING_INFO(interfaceLogger, " Clearing obstacles ... " <<endl);
  Environment::getInstance()->clearObstacles( DEPTH );
  LOGGING_INFO(interfaceLogger, " Process depth ... " <<endl);
  mProcessSensor.processDepthSensor(image);

  if( mDebugActive )
  {
    LOGGING_INFO(interfaceLogger, " Write depth debug info ... " <<endl);
    std::stringstream sstr;
#ifdef noPng
    sstr << mDebugOutputPath << "/Depth/" << std::setfill('0') << std::setw(5) << mImageCounter << ".bmp";
#else
    sstr << mDebugOutputPath << "/Depth/" << std::setfill('0') << std::setw(5) << mImageCounter << ".png";
#endif
    cv::imwrite( sstr.str(), image );
    //cv::imwrite( "debugDepth.png", mProcessSensor.getDepthImageProcessor()->getDebugImage( image ) );	// TODO: Remove
  }
  LOGGING_INFO(interfaceLogger, " Depth image is done as well. " <<endl);
}

void Interface::setCarPose( const ExtendedPose2d &carPose )
{
  LOGGING_INFO(interfaceLogger, " Updating car pose ... " <<endl);
  mDriver.update( carPose );

  // Comment out if trajectory debugging is not needed:
  if( mDebugActive )
    mLogTrajectory.push_back( carPose );

  LOGGING_INFO(interfaceLogger, " Car pose updated ." <<endl);
}

void Interface::setUsSensor(oadrive::obstacle::usSensor measurements )
{
  LOGGING_INFO(USDataLogger,"UsData:" << endl <<
      "\tfL: " << measurements.frontLeft <<
      "\tfCl: " << measurements.frontCenterLeft <<
      "\tfC: " << measurements.frontCenter <<
      "\tfCr: " << measurements.frontCenterRight <<
      "\tfR: " << measurements.frontRight <<
      "\tsL: " <<  measurements.sideLeft <<
      "\tsR: " << measurements.sideRight <<
      "\trL: " << measurements.rearLeft <<
      "\trC: " << measurements.rearCenter <<
      "\trR: " << measurements.rearRight <<
      "\tFrame: " << mImageCounter << endl );
  mProcessSensor.setNewUsSensorValues( measurements );


  LOGGING_INFO(interfaceLogger, " US data processed ... " <<endl);
}

float Interface::getSteering()
{
  //return M_PI/10.0f;	// debug angle, slight left turn
  return mDriver.getSteeringAngle();
}

float Interface::getSpeed()
{
  float returnSpeed = mDriver.getSpeed();
  LOGGING_INFO(interfaceLogger, "Speed: " << returnSpeed << endl);

  // are we breaking or stopping
    if(mLastSpeed - returnSpeed > 0.1 || returnSpeed == 0) {
      if(!mBrakeLightsOn) {
        // turn on breaking lights
        setLights(BRAKE_LIGHT, 1);
        mBrakeLightsOn = true;
      }
    }
    else {
      if(mBrakeLightsOn) {
        // turn off breaking lights
        setLights(BRAKE_LIGHT, 0);
        mBrakeLightsOn = false;
      }
    }

    // are we driving backwards?
    if(returnSpeed < 0) {
      if(!mReverseLightsOn) {
        // turn on reverse lights
        setLights(REVERSE_LIGHT, 1);
        mReverseLightsOn = true;
      }
    }
    else {
      if(mReverseLightsOn) {
        // turn off reverse lights
        setLights(REVERSE_LIGHT, 0);
        mReverseLightsOn = false;
      }
    }

    // set last speed to current speed
    mLastSpeed = returnSpeed;


  LOGGING_INFO(interfaceLogger, " got Speed " <<endl);
  return returnSpeed;
}

void Interface::setJuryCommand( juryActions action, int maneuverEntryID )
{

  LOGGING_INFO(interfaceLogger, " Setting Jury command ... " <<endl);
  JuryCommand cmd;
  cmd.action = action;
  cmd.maneuverEntryID = maneuverEntryID;

  mtx.lock();
  mJuryCommandQueue.push_back( cmd );
  mtx.unlock();

  LOGGING_INFO(interfaceLogger, " got jury command " <<endl);
}

void Interface::processJuryCommands()
{

  LOGGING_INFO(interfaceLogger, " Process jury ... " <<endl);
  mtx.lock();
  for( std::vector<JuryCommand>::iterator it = mJuryCommandQueue.begin();
      it != mJuryCommandQueue.end(); it ++ )
  {
    mMissionControl.eventJurySignalReceived( it->action, it->maneuverEntryID );
  }
  mJuryCommandQueue.clear();
  mtx.unlock();
  LOGGING_INFO(interfaceLogger, " Jury processed ... " <<endl);
}

void Interface::setManeuverList( std::string list )
{
  LOGGING_INFO(interfaceLogger, " Setting manoveur list ... " <<endl);
  mtx.lock();
  mManeuverList = list;
  mNewManeuverlistReceived = true;
  mtx.unlock();
  LOGGING_INFO(interfaceLogger, " Manoveur list set. " <<endl);
}

void Interface::processNewManeuverList()
{

  LOGGING_INFO(interfaceLogger, "processing man lst ... " <<endl);
  mtx.lock();
  if( mNewManeuverlistReceived )
  {
    mNewManeuverlistReceived = false;
    mMissionControl.eventManeuverListReceived( mManeuverList );
    if( mDebugActive )
    {
      std::string fileName = mDebugOutputPath;
      fileName.append( "/maneuverList.xml" );
      std::ofstream outFile( fileName.c_str() );
      outFile << mManeuverList;
      outFile.close();
    }
  }
  mtx.unlock();
  LOGGING_INFO(interfaceLogger, "Man list processed ... " <<endl);
}

void Interface::incrementTime(int milliSeconds)
{
  mTimer.setTimeIncrement(milliSeconds);
}


/////////////////////////////////////////////////////////////
// Handle MissionControl Events:

void Interface::setJuryState( stateCar state, int manID )
{
  LOGGING_INFO( interfaceLogger, "New state received from MissionControl: " << state << " (maneuver list entry: " << manID << ")" << endl );
  // If an event handler has been registered, pass on the message:
  if( mIControl4MC )
  {
    mIControl4MC->setJuryState( state, manID );
  } else {
    LOGGING_INFO( interfaceLogger,"Ignoring, because no filter is registered. " << endl );
  }
}

void Interface::setLights( enumLight light, bool on )
{
  //lightMtx.lock();
  LOGGING_INFO( interfaceLogger, "Set lights command: " <<
                light << (on ? " on" : " off") << endl );
  // If an event handler has been registered, pass on the message:
  if( mIControl4MC )
  {
    mIControl4MC->setLights( light, on );
  } else {
    LOGGING_INFO( interfaceLogger,"Ignoring, because no filter is registered. " << endl );
  }
  //lightMtx.unlock();
}



/////////////////////////////////////////////////////////////
void Interface::startDebugDumping( std::string folder )
{
  // Make sure the directories exist:
  LOGGING_INFO( interfaceLogger, "[Interface] Generating debug path: " << folder << endl );
  create_directories( folder );
  create_directories( folder + "/Map" );
  create_directories( folder + "/Features" );
  create_directories( folder + "/Trajectories" );
  create_directories( folder + "/Depth" );

  //	std::ifstream source( mBirdViewCalFile.c_str() );
  //	path destName( folder );
  //	//destName /= mCarName;
  //	destName /= "BirdviewCal.yml";
  //	std::ofstream dest( destName.string().c_str() );
  //	dest << source.rdbuf();
  //	source.close();
  //	dest.close();
  copyDir(mConfigPath,folder+"/config");


  mDebugActive = true;
  mDebugOutputPath = folder;
  LOGGING_INFO( interfaceLogger, "[Interface] Copied config files to " << folder + "/Config" << "." << endl );

  //mTrajectoryFactory.startDebugDumping( folder + "/Trajectories/" );
}

void Interface::dumpDebugData( const ExtendedPose2d &currentCarPose, cv::Mat image )
{
  std::string filename( mDebugOutputPath );
  filename.append( "/carPose.txt" );
  std::ofstream file( filename.c_str(), std::ios_base::app | std::ios_base::binary );
  double x = currentCarPose.getX();
  double y = currentCarPose.getY();
  double yaw = currentCarPose.getYaw();
  file.write(reinterpret_cast<const char*>(&x), sizeof(double));
  file.write(reinterpret_cast<const char*>(&y), sizeof(double));
  file.write(reinterpret_cast<const char*>(&yaw), sizeof(double));
  file.close();

  //file << pose.getX() << " " << pose.getY() << " " << pose.getYaw() << std::endl;
  // Generate image name. Should be zero-padded!
  std::stringstream sstr;
#ifdef noPng
  sstr << mDebugOutputPath << "/" << std::setfill('0') << std::setw(5) << mImageCounter << ".bmp";
#else
  sstr << mDebugOutputPath << "/" << std::setfill('0') << std::setw(5) << mImageCounter << ".png";
#endif
  //cv::Mat featureImage = generateDebugFeatureImage();
  cv::imwrite( sstr.str(), image );

  sstr.clear();
  sstr.str("");
#ifdef noPng
  sstr << mDebugOutputPath << "/Features/" << std::setfill('0') << std::setw(5) << mImageCounter << ".bmp";
#else
  sstr << mDebugOutputPath << "/Features/" << std::setfill('0') << std::setw(5) << mImageCounter << ".png";
#endif
//  cv::Mat featureImage = generateDebugFeatureImage();
//  cv::imwrite( sstr.str(), featureImage );

  sstr.clear();
  sstr.str("");
#ifdef noPng
  sstr << mDebugOutputPath << "/Map/" << std::setfill('0') << std::setw(5) << mImageCounter << ".bmp";
#else
  sstr << mDebugOutputPath << "/Map/" << std::setfill('0') << std::setw(5) << mImageCounter << ".png";
#endif
/*  cv::Mat map = Environment::getInstance()->getEnvAsImage(
      currentCarPose.getX(), currentCarPose.getY(), 2, 50 );
  cv::imwrite( sstr.str(), map );*/

  sstr.str("");
  sstr.clear();
  frameTimer.stop();
  LOGGING_INFO(interfaceLogger,"write out debug image number: " << mImageCounter << " (" << 1. / frameTimer.value << " fps)" << endl);
  frameTimer.reset();
  frameTimer.start();
  mImageCounter ++;
}

cv::Mat Interface::generateDebugFeatureImage()
{
  cv::Mat channels[4];
  //cv::cvtColor( mLastBirdViewImage, output, CV_GRAY2BGR );

  cv::Mat features = mStreetPatcher.generateDebugImage();
  cv::split( features, channels );
  cv::Mat sum;
  cv::bitwise_or( mLastBirdViewImage, 0, sum, 255 - channels[3] );
  //std::vector<cv::Mat> featuresChannels;
  //featuresChannels.push_back( channels[0] );
  //featuresChannels.push_back( channels[1] );
  //featuresChannels.push_back( channels[2] );
  cv::Mat featuresRGB;
  cv::cvtColor( features, featuresRGB, CV_BGRA2BGR );
  sum = sum + featuresRGB;

  return sum;
}

cv::Mat Interface::generateDebugMap( int width, int height, float mapSize )
{
  EnvironmentPtr env = Environment::getInstance();
  // Generate map, large enough to fit width and height:
  cv::Mat map = env->getEnvAsImage(
        env->getCarPose().getX(), env->getCarPose().getY(),
        mapSize, std::max( width, height )/(2*mapSize) );

  // Return center of the map:
  int diffX = (map.size().width - width)/2;
  int diffY = (map.size().height - height)/2;
  cv::Mat roi( map, cv::Rect( diffX, diffY, width, height));
  return roi;
}

bool Interface::copyDir(
    std::string sourcestr,std::string destinationStr
    )
{
  boost::filesystem::path  source(sourcestr);
  boost::filesystem::path destination (destinationStr);
  namespace fs = boost::filesystem;
  try
  {
    // Check whether the function call is valid
    if(
       !fs::exists(source) ||
       !fs::is_directory(source)
       )
    {
      std::cerr << "Source directory " << source.string()
                << " does not exist or is not a directory." << '\n'
                   ;
      return false;
    }
    if(fs::exists(destination))
    {
      std::cerr << "Destination directory " << destination.string()
                << " already exists." << '\n'
                   ;
      return false;
    }
    // Create the destination directory
    if(!fs::create_directory(destination))
    {
      std::cerr << "Unable to create destination directory"
                << destination.string() << '\n'
                   ;
      return false;
    }
  }
  catch(fs::filesystem_error const & e)
  {
    std::cerr << e.what() << '\n';
    return false;
  }
  // Iterate through the source directory
  for(
      fs::directory_iterator file(source);
      file != fs::directory_iterator(); ++file
      )
  {
    try
    {
      fs::path current(file->path());
      if(fs::is_directory(current))
      {
        // Found directory: Recursion
        if(
           !copyDir(
             current.string(),
             (destination / current.filename()).string()
             )
           )
        {
          return false;
        }
      }
      else
      {
        // Found file: Copy
        fs::copy_file(
              current,
              destination / current.filename()
              );
      }
    }
    catch(fs::filesystem_error const & e)
    {
      std:: cerr << e.what() << '\n';
    }
  }
  return true;
}


cv::Mat Interface::getLastBirdViewImage()
{
  return mLastBirdViewImageGray;
}


void Interface::eventTimerFired( timerType type, unsigned long timerID )
{
  if( type == TIMER_TYPE_SEND_MAP )
  {
    if( Broker::isActive() )  // check if we're currently connected (i.e. Kuer is active)
    {
      // If so, send the current map:
      EnvironmentPtr env = Environment::getInstance();
      Broker::getInstance()->publish(CHANNEL_SEND_MAP, env->toJson(2.0));
    }
    mTimer.setTimer( 1000, TIMER_TYPE_SEND_MAP );
  }
}

}   // namespace
}   // namespace

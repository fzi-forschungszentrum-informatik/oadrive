/*! Tests the driver module, which is used to hold and drive a trajectory.
 * Test written, because if the trajectory reaches an angle of 180°, the
 * lateral controller suddenly spits out very weird angles (180° jump?)
 *
 * Usage: run the test, hold down any key to drive, ESC to close." */

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <oadrive_control/DriverModule.h>
#include <oadrive_control/LateralController.h>
#include <oadrive_world/TrajectoryFactory.h>
#include <oadrive_util/Config.h>

using namespace oadrive::core;
using namespace oadrive::control;
using namespace oadrive::world;
using namespace oadrive::util;

int main(int argc, char** argv )
{
  //   Trajectory2d traj = DriverModule::generateTestTrajectory( "halfCircle" );
  //   Trajectory2d traj = DriverModule::generateTestTrajectory( "rectangle" );
  Trajectory2d traj;

  if ( argc < 4 )
  {
    std::cout << "Usage: test_oadrive_driermodule <configPath> <CarName> <trajectory>" << std::endl;
    std::cout << "\tWhere <configPath> is a folder containing the config" << std::endl;
    std::cout << "\tCarName is an optional car name (default: Goffin)" << std::endl;
    std::cout << "\ttrajectory is an optional trajectory file name" << std::endl;
    return 1;
  }

  std::string configFolder = argv[1];
  configFolder.append( "config" );
  std::string carName = "Goffin";

  if( argc >= 3 )
  {
    carName = argv[2];
  }
  if( argc >= 4 )
  {
    std::string filename( argv[2] );
    std::ifstream trajFile( filename.c_str() );
    trajFile >> traj;
    trajFile.close();
    std::cout << "Loaded trajectory:" << std::endl;
    std::cout << traj << std::endl;
    std::cout << "(forward: " << traj.isForwardTrajectory() << ")" << std::endl;
  } else {
    traj = TrajectoryFactory::generateTestTrajectory( "oval" );
  }
  
  std::cout << "Config folder: " << configFolder << std::endl;
  std::string mConfigPath( Config::setConfigPath( configFolder, carName ) );

  MultiTrajectory multiTraj;
  multiTraj.trajectories.push_back( traj );

  DriverModule driver;
  driver.setTrajectory( multiTraj );
  driver.setTargetSpeed(0.3);
  driver.drive();
  //traj = driver.getTrajectory();
  //traj.toGnuplot( "/tmp/trajCircle" );

  std::cout << "-------------------------------------------------" << std::endl;
  std::cout << "Usage: Hold down space key to drive. ESC to quit." << std::endl;
  std::cout << "-------------------------------------------------" << std::endl;

  ExtendedPose2d carPose, prevPose;

  float meters2Pixels = 50;
  int center = 300;

  cv::Mat img( cv::Size( 600, 600 ), CV_8UC3 );
  for( size_t i = 0; i < traj.size(); i++ )
  {
    cv::Point p( traj[i].getX()*meters2Pixels + center, traj[i].getY()*meters2Pixels + center );
    cv::circle( img, p, 3, cv::Scalar( 64, 64, 64 ) );
  }
  cv::Mat overlay( cv::Size( 600, 600 ), CV_8UC3 );

  //carPose.setY( 0.3 );
  //carPose.setX( .3 );

  while( true )
  {
    float dt = 0.15;		// delta time

    driver.update( carPose );

    float steering = M_PI/180.0*driver.getSteeringAngle();
    float speed = driver.getSpeed();

    std::cout << "Speed: " << speed << std::endl;
    std::cout << "Angle: " << steering << std::endl;

    prevPose = carPose;

    carPose.setYaw( carPose.getYaw() - steering*dt );
    carPose.setX( carPose.getX() + speed*dt*cos( carPose.getYaw() ) );
    carPose.setY( carPose.getY() + speed*dt*sin( carPose.getYaw() ) );

    {
      cv::Point prev( prevPose.getX()*meters2Pixels + center,
                      prevPose.getY()*meters2Pixels + center );
      cv::Point current( carPose.getX()*meters2Pixels + center,
                         carPose.getY()*meters2Pixels + center );
      cv::line( img, prev, current, cv::Scalar( 255, 255, 255 ) );
    }

    overlay = cv::Scalar( 0, 0, 0 );
    {
      cv::Point direction( meters2Pixels*speed*cos(carPose.getYaw()),
                           meters2Pixels*speed*sin(carPose.getYaw()));
      cv::Point current( carPose.getX()*meters2Pixels + center,
                         carPose.getY()*meters2Pixels + center );
      cv::line( overlay, current, current + direction, cv::Scalar( 255, 128, 0 ) );

      /*ExtendedPose2d projection = driver.getLateralController()->getProjectedPose();
      cv::Point p( projection.getX()*meters2Pixels + center,
                   projection.getY()*meters2Pixels + center );
      cv::circle( overlay, p, 2, cv::Scalar( 64, 64, 255 ) );

      cv::Point directionProj( meters2Pixels*speed*cos( projection.getYaw()),
                               meters2Pixels*speed*sin( projection.getYaw() ));*/
    }

    cv::Mat output = img*0.5 + overlay;
    cv::Mat flipped;
    cv::flip( output, flipped, 0 );

    imshow( "trajectory", flipped );

    int code = cv::waitKey(1048603);
    //std::cout<<"KeyCode: "<<code<<std::endl;
    if ( code == 27 )		// ESC to abort
      return 1;
    if (code == 1048676 )
      driver.drive();
    if(code == 1048691)
      driver.halt();
  }

  return 0;
}

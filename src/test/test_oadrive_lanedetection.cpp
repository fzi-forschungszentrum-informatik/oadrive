#include <oadrive_lanedetection/StreetPatcher.h>
#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_lanedetection/HaarFilter.h>
//#include <oadrive_trajectory/TrajUtil.h>
#include <oadrive_world/Environment.h>
#include <oadrive_missioncontrol/MissionControl.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/flann/timer.h>
#include <iostream>
#include <fstream>

using namespace oadrive::core;
using namespace oadrive::lanedetection;
using namespace oadrive::util;
//using namespace oadrive::trajectory;
using namespace oadrive::world;
using namespace oadrive::missioncontrol;

int main(int argc, char** argv)
{
  if ( argc == 2 )
  {
    printf("ESC to quit, any other key to continue to next image.... xD \n");

    std::vector<cv::String> filenames;
    cv::String folder = cv::String(argv[1]);

    cv::glob(folder, filenames);

    CoordinateConverter converter;
    std::string filename = folder;
    filename.append( "/BirdviewCal.yml" );
    std::ifstream calibFile( filename.c_str() );
    if( calibFile.is_open() )
    {
      calibFile.close();
      converter.readConfigFile( filename );
    }

    cvflann::StartStopTimer timer;
    EnvironmentPtr environment = Environment::init();

    StreetPatcher patcher( 2.5, &converter );

    std::vector<ExtendedPose2d> carPoses;
    // Check if there is a carPose.txt in the folder. If so, open it and read the car poses:
    filename = folder;
    filename.append( "/carPose.txt" );
    std::ifstream file( filename.c_str() );
    if( file.is_open() )
    {
      std::cout << "Found car pose file!" << std::endl;
      float x, y, yaw;
      while( file >> x >> y >> yaw )
      {
        carPoses.push_back( ExtendedPose2d( x, y, yaw ) );
      }
    } else {
      std::cout << "WARNING: Car pose file not found." << std::endl << "\tWill use 0,0,0 as car pose for every frame." << std::endl;
    }

    bool animate = false;

    for(size_t i = 0; i < filenames.size(); ++i)
    {
      ExtendedPose2d carPose( 0,0,0 );
      if( carPoses.size() > i )
        carPose = carPoses[i];

      std::cout << filenames[i] << std::endl;
      std::cout << "Car pose: " << carPose.getX() << " " << carPose.getY() << " " << carPose.getYaw() << std::endl;
      environment->updateCarPose( carPose );

      //cv::Mat image = imread( filenames[i], CV_BGR2GRAY );
      //cv::Mat loadedGray = cv::imread( filenames[i], CV_LOAD_IMAGE_GRAYSCALE );
      cv::Mat newImage = cv::imread( filenames[i] );
      if(! newImage.data )
      {
        cv::waitKey(0);
        return 1;
      }
      cv::Mat image;
      cv::cvtColor( newImage, image, CV_BGR2GRAY );
      //cv::Mat image = loadedGray;

      cv::Mat output, channels[4];
      cv::cvtColor( image, output, CV_GRAY2BGRA );

      /*imshow( "loadedGray", loadedGray );
            imshow( "converted", image );
            imshow( "difference", image - loadedGray );*/

      // Find patches:
      timer.reset(); timer.start();
      patcher.setImage( image, carPose );
      timer.stop();
      imshow( "input", image );

      // Get current trajectory:
      //Trajectory2d traj = environment->getTrajectory();

      cv::Mat features = patcher.generateDebugImage();
      cv::split( features, channels );
      cv::Mat sum;
      cv::bitwise_or( output, 0, sum, 255 - channels[3] );
      sum = sum + features;
      imshow( "output", sum );

      cv::Mat houghSpace = patcher.generateHoughSpaceDebugImage();
      imshow( "Hough Space", houghSpace );

      cv::Mat map = environment->getEnvAsImage( carPose.getX(), carPose.getY(), 7, 50 );
      imshow( "map", map );

      std::cout << "Time taken for image: " << timer.value << std::endl;

      int code;
      if( animate )
        code = cv::waitKey(1);
      else
        code = cv::waitKey(0);

      if ( code == 27 )		// ESC to abort
        return 1;
      else if ( code == 65 || code == 97 )
        animate = !animate;
    }

  } else {
    std::cout << "usage: test_oadrive_lanedetection <Path>" << std::endl;
    std::cout << "\t(where <Path> is a folder with bird view images in it)" << std::endl;
  }
  return 0;

}

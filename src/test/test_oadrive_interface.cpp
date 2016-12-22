#include <oadrive_lanedetection/StreetPatcher.h>
#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_lanedetection/HaarFilter.h>
#include <oadrive_world/Environment.h>
#include <oadrive_interface/Interface.h>
#include <oadrive_obstacle/ProcessUS.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/regex.hpp>
#include <string>

using namespace oadrive::core;
using namespace oadrive::lanedetection;
using namespace oadrive::util;
using namespace oadrive::world;
using namespace oadrive::interface;
using namespace oadrive::missioncontrol;
using namespace boost;

//#define compileOnCar
#define TIME_PER_FRAME 100

struct usDataWithFrame
{
  oadrive::obstacle::usSensor usDataRaw;
  size_t frame;
};

void showDebugOutput( Interface* interface, cv::Mat image,cv::Mat depthImage, ExtendedPose2d carPose )
{

    //generate Debug Images
  cv::Mat features = interface->generateDebugFeatureImage();
  cv::Mat houghSpace = interface->getPatcher()->generateHoughSpaceDebugImage();

  cv::Mat map =
    Environment::getInstance()->getEnvAsImage( carPose.getX(), carPose.getY(), 3, 100 );

#ifndef compileOnCar
  imshow( "input", image );
  imshow( "output", features );
  imshow( "Hough Space", houghSpace );
  imshow( "map", map );
  if(!depthImage.empty())
  {
    imshow("CleanDepthImage",depthImage);
  }

#endif

}

bool readUsDataLine(std::string stringData, usDataWithFrame &usData)
{
  std::size_t pos = stringData.find("Frame: ");
  if (pos != std::string::npos)
  {
    //this is the right line
    //format:  [2016-03-09 15:03:12]	fL: 4.230000	fCl: 0.000000	fC: 4.230000	fCr: 0.000000	fR: 0.000000	sL: 0.000000	sR: 0.000000	rL: 0.000000	rC: 0.000000	rR: 0.000000	Frame: 0
    std::stringstream data;
    std::size_t pos;

    pos = stringData.find("fL: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: fL:"<<std::endl;
      throw;
    }
    usData.usDataRaw.frontLeft = std::atof(stringData.substr(pos+4,8).c_str());;

    pos = stringData.find("fCl: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: fCl:"<<std::endl;
      throw;
    }
    usData.usDataRaw.frontCenterLeft = std::atof(stringData.substr(pos+5,8).c_str());;

    pos = stringData.find("fC: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: fC: "<<std::endl;
      throw;
    }
    usData.usDataRaw.frontCenter = std::atof(stringData.substr(pos+4,8).c_str());;

    pos = stringData.find("fCr: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: fCr: "<<std::endl;
      throw;
    }
    usData.usDataRaw.frontCenterRight = std::atof(stringData.substr(pos+5,8).c_str());;

    pos = stringData.find("fR: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: fR: "<<std::endl;
      throw;
    }
    usData.usDataRaw.frontRight= std::atof(stringData.substr(pos+4,8).c_str());;

    pos = stringData.find("sL: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: sL: "<<std::endl;
      throw;
    }
    usData.usDataRaw.sideLeft = std::atof(stringData.substr(pos+4,8).c_str());;

    pos = stringData.find("sR: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: sR: "<<std::endl;
      throw;
    }
    usData.usDataRaw.sideRight = std::atof(stringData.substr(pos+4,8).c_str());

    pos = stringData.find("rL: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: rL: "<<std::endl;
      throw;
    }
    usData.usDataRaw.rearLeft = std::atof(stringData.substr(pos+4,8).c_str());

    pos = stringData.find("rC: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: rC: "<<std::endl;
      throw;
    }
    usData.usDataRaw.rearCenter = std::atof(stringData.substr(pos+4,8).c_str());

    pos = stringData.find("rR: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: rR: "<<std::endl;
      throw;
    }
  usData.usDataRaw.rearRight = std::atof(stringData.substr(pos+4,8).c_str());

    pos = stringData.find("Frame: ");
    if (pos == std::string::npos)
    {
      std::cerr<<"error in reading Us File. Can't find: Frame: "<<std::endl;
      throw;
    }
    usData.frame = std::atoi(stringData.substr(pos+7,stringData.length()-(pos+7)).c_str());;
//    std::cout<<"Original Data:"<<stringData<<std::endl;
//    std::cout<<"UsDataReread:" <<
//        "\tfL: " << usData.usDataRaw.frontLeft <<
//        "\tfCl: " << usData.usDataRaw.frontCenterLeft <<
//        "\tfC: " << usData.usDataRaw.frontCenter <<
//        "\tfCr: " << usData.usDataRaw.frontCenterRight <<
//        "\tfR: " << usData.usDataRaw.frontRight <<
//        "\tsL: " <<  usData.usDataRaw.sideLeft <<
//        "\tsR: " << usData.usDataRaw.sideRight <<
//        "\trL: " << usData.usDataRaw.rearLeft <<
//        "\trC: " << usData.usDataRaw.rearCenter <<
//        "\trR: " << usData.usDataRaw.rearRight <<
//        "\tFrame: " << usData.frame << std::endl;
    return true;
  }
  return false;
}

int main(int argc, char** argv)
{
  try{

    if ( argc > 1 )
    {
      printf("ESC to quit,a for automatic stepping, r for reset, s for new initial vote and any other key to continue to next image.\n");

      std::vector<cv::String> filenames;
      cv::String folder = cv::String(argv[1]);
      cv::String pattern = folder;
      pattern += cv::String( "/*.png" );

      cv::glob(pattern, filenames);

      unsigned int startImage = 0;

      std::string carName = "Goffin";
      std::cout << "NUMBER OF ARGUMENTs: " << argc << std::endl;
      if( argc > 2 )
      {
        carName = std::string(argv[2]);
      }
      std::string configFolder = folder;
      configFolder.append("/config/");

      if( argc > 3 )
      {
        std::stringstream sstr;
        sstr << argv[3];
        int numImages;
        sstr >> numImages;
        startImage = numImages;
      }

      Interface interface( configFolder, carName );
      interface.startDebugDumping( "/tmp/recordedData/" );
      interface.getStreetPatcher()->setSearchParkingLots( false );

      // Look for a maneuverlist in the given directory:

      std::string fileName = folder;
      fileName.append("/maneuverList.xml");
      std::ifstream maneuverListFile( fileName.c_str() );
      std::cout << "Trying to load maneuverlist: " << fileName << std::endl;
      if( maneuverListFile.is_open() )
      {
        std::stringstream strStream;
        strStream << maneuverListFile.rdbuf();//read the file
        interface.setManeuverList( strStream.str() );
        std::cout << "Opened maneuverlist: " << strStream.str() << std::endl;
      } else {
        std::cout << "Could not open maneuerList.xml." << std::endl;
      }
        std::cout << "Send Jury Get Ready" << std::endl;
      interface.setJuryCommand( action_GETREADY, 0 );

      std::vector<ExtendedPose2d> carPoses;
      // Check if there is a carPose.txt in the folder. If so, open it and read the car poses:
      std::string filename = folder;
      filename.append( "/carPose.txt" );
//      car poses are written in binary

        std::ifstream file( filename.c_str(), std::ios::binary );
        if( file.is_open() )
        {
          std::cout << "Found car pose file (binary)!" << std::endl;
          double x, y, yaw;
          char buffer[sizeof(double)*3];

          while( ! file.eof() )
          {
            file.read( buffer, sizeof(double)*3 );
            memcpy(&x, &buffer[0], sizeof(double));
            memcpy(&y, &buffer[sizeof(double)], sizeof(double));
            memcpy(&yaw, &buffer[sizeof(double)*2], sizeof(double));
            carPoses.push_back( ExtendedPose2d( x, y, yaw ) );
          }
        } else {
          std::cout << "WARNING: Car pose file not found." << std::endl << "\tWill use 0,0,0 as car pose for every frame." << std::endl;
        }


      std::string filenameUsData = folder;
      filenameUsData.append("/usLog.txt");
      std::ifstream fileUsData( filenameUsData.c_str() );
      std::vector <usDataWithFrame> usSensors;
      if(fileUsData.is_open())
      {
          std::cout<<"Found Us Data file."<<std::endl;
          std::string line;
          while(std::getline(fileUsData,line))
          {
            usDataWithFrame usData;
            if(readUsDataLine(line,usData))
            {
              usSensors.push_back(usData);
            }
          }
          std::cout<<"Read "<<usSensors.size()<<" sets of Us data"<<std::endl;

      }
      else
      {
          std::cout<<"Can't open Usdata file!"<<std::endl;
      }


      bool animate = false;
#ifndef compileOnCar
      namedWindow("map",cv::WINDOW_NORMAL);
      cv::resizeWindow("map",600,600);
#endif

      regex filenamePattern(".*/(.*?)");

      size_t usSampleNumber = 0;
      bool usDataEnd = false;
      for(size_t i = startImage; i < filenames.size(); ++i)
      {
        ExtendedPose2d carPose( 0,0,0 );
        std::cout << i << " poses: " << carPoses.size() << std::endl;
        if( carPoses.size() > i )
          carPose = carPoses[i];
        if(usSensors.size()>0&&!usDataEnd)
        {
          std::cout<<"US Frame: "<<usSensors[usSampleNumber].frame<<"UsSampleNumber: "<<usSampleNumber<<std::endl;
          while(usSensors[usSampleNumber].frame == i)
          {
            interface.setUsSensor(usSensors[usSampleNumber].usDataRaw);
            std::cout<<"Read us Data for Frame: "<<i<<std::endl;
            if(usSampleNumber < usSensors.size()-1)
            {
              usSampleNumber++;
            }
            else
            {
              usDataEnd = true;
              break; //No more US Data available
            }
          }
        }

        std::cout << filenames[i] << std::endl;
        std::cout << "Car pose: " << carPose;


        /*if( i == startImage )
        {
          interface.getStreetPatcher()->setAPrioriVote(
                ExtendedPose2d( carPose.getX() + 1,
                                carPose.getY() + 0.22, carPose.getYaw()) );
        }*/

        cv::Mat image = cv::imread( filenames[i] );
        cv::Mat depthImageDebug;
        if(! image.data )
        {
#ifndef compileOnCar
          cv::waitKey(0);
#endif
          std::cout<<"Failed to read file!" <<filenames[i]<<std::endl;
          std::cout<<"Last file was: "<<filenames[i-1]<<std::endl;
          break;
        }
        interface.setCarPose( carPose );

        std::cout << "Steering: " << interface.getSteering() <<
                     " Speed: " << interface.getSpeed() << std::endl;

        // Find patches and update street:
        interface.setCameraImage( image, false );


        boost::match_results<std::string::const_iterator> res;
        std::string str = filenames[i];
        if(regex_match( str, res, filenamePattern ))
        {
          std::string name = res[1];
          std::string fullDepthName = folder;
          fullDepthName += "/Depth/";
          fullDepthName += name;
          cv::Mat image = cv::imread( fullDepthName, CV_LOAD_IMAGE_ANYDEPTH|CV_LOAD_IMAGE_GRAYSCALE);
          if( image.data )
          {

            interface.setDepthImage( image );
            depthImageDebug = interface.getDepthImageProcessor()->getDebugImage(image);

          }

        }

#ifndef compileOnCar

        // Draw everything on screen:
        showDebugOutput( &interface, image,depthImageDebug, carPose );

        int code;
        if( animate )
          code = cv::waitKey(1);
        else
          code = cv::waitKey(0);

        if ( char(code) == 27 )		// ESC to abort
        {
          break;
        }
        else if ( char(code) == 82 || char(code) == 114 )		// r to reset
        {
          //interface.getPatcher()->reset();
          //Environment::getInstance()->clearAllPatches();
          std::cout << "Send Jury Stop" << std::endl;
          interface.setJuryCommand( action_STOP, 1 );
          std::cout << "Send Jury Start" << std::endl;
          interface.setJuryCommand( action_START, 1 );

          interface.reset();
          showDebugOutput( &interface, image, depthImageDebug, carPose );
          code = cv::waitKey(0);
        }
        else if ( char(code) == 65 || char(code) == 97 )	// a to animate
        {
          animate = !animate;
        }
        else if ( char(code) == 83 || char(code) == 115 )		// s to reset
        {
          ExtendedPose2d carPose = Environment::getInstance()->getCarPose();
          double dx = 1.0;		// 1 meter in front of me
          double dy = 0.23;		// 0.23 meters next me
          ExtendedPose2d votePose(
              carPose.getX() + dx*cos( carPose.getYaw() ) - dy*sin( carPose.getYaw() ),
              carPose.getY() + dx*sin( carPose.getYaw() ) + dy*cos( carPose.getYaw() ),
              carPose.getYaw() );
          interface.getStreetPatcher()->setAPrioriVote( votePose );

        }
#endif
        if(i == startImage){
            std::cout << "Send Jury Start" << std::endl;
            interface.setJuryCommand( action_START, 0 );
        }
      }


    } else {
      std::cout << "usage: test_oadrive_interface <Path> carname startnumber" << std::endl;
      std::cout << "\t(where <Path> is a folder with bird view images in it," << std::endl;
      std::cout << "\t carname is Goffin or Inka" << std::endl;
      std::cout << "\tand startnumber is the image at which to start (optional))" << std::endl;
    }

    return 0;
  } catch( const char* msg ) {
    std::cout << "Error: " << msg << std::endl;
    return 1;
  }
}



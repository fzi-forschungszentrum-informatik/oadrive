#include <oadrive_lanedetection/HaarFilter.h>
#include <opencv2/flann/timer.h>
#include <oadrive_util/Config.h>
#include <oadrive_util/BirdViewConverter.h>
#include <boost/filesystem.hpp>

using namespace oadrive::lanedetection;
using namespace oadrive::util;

int main(int argc, char** argv )
{
  if ( argc >= 2 )
  {
    std::cout << "ESC to quit, any other key to continue to next image." << std::endl;

    std::vector<cv::String> filenames;
    cv::String folder = cv::String(argv[1]);
    cv::glob(folder, filenames);
 
    std::string configFolder = argv[1];
    configFolder.append( "config" );
    std::string carName = "Goffin";
    if( argc >= 3 )
    {
      carName = argv[2];
    }
    
    std::string mConfigPath( Config::setConfigPath( configFolder, carName ) );
    std::string mBirdViewCalFile( ( boost::filesystem::path(configFolder) / carName / "BirdviewCal.yml").string() );

    CoordinateConverter converter( mConfigPath );
    HaarFilter filter( &converter );

    BirdViewConverter mBirdViewConverter;
    mBirdViewConverter.loadConfig( mBirdViewCalFile );

    cvflann::StartStopTimer timer;

    for(size_t i = 0; i < filenames.size(); ++i)
    {
      cv::Mat image = cv::imread( filenames[i], CV_LOAD_IMAGE_GRAYSCALE );
      std::cout << filenames[i] << std::endl;

      cv::Mat mBirdViewImageGray = mBirdViewConverter.transform( image );
      cv::Mat mBirdViewImage;
      cv::cvtColor(mBirdViewImageGray, mBirdViewImage, CV_GRAY2BGR);

      timer.start();

      filter.setImage( mBirdViewImageGray );

      filter.calculateFeaturesPerLine(
          true,
          true);

      FeatureVector* featureVec = filter.getFeatures();
      std::cout << featureVec->size() << std::endl;

      timer.stop();
      std::cout << "Time taken for image: " << timer.value << std::endl;

      //cv::Mat result = filter.generateDebugImage();

      cv::Mat channels[4];
      //cv::cvtColor( mLastBirdViewImage, output, CV_GRAY2BGR );

      cv::Mat features = filter.generateDebugImage();
      cv::split( features, channels );
      cv::Mat sum;
      cv::bitwise_or( mBirdViewImage, 0, sum, 255 - channels[3] );

      cv::Mat featuresRGB;
      cv::cvtColor( features, featuresRGB, CV_BGRA2BGR );
      sum = sum + featuresRGB;

      imshow( "input", mBirdViewImageGray );
      imshow( "result", sum );

      int code = cv::waitKey(0);
      if ( code == 27 )		// ESC to abort
        return 1;
    }
  }
  else
  {
    std::cout << "Usage: test_oadrive_haarfeatures <path> [CarName]" << std::endl;
    std::cout << "\tWhere <path> is a folder containing a test case and" << std::endl;
    std::cout << "\tCarName is an optional car name (default: Goffin)" << std::endl;
  }
  return 0;
}


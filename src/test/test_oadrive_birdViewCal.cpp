#include "oadrive_util/CameraCalibration.h"
#include "iostream"
enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2, WAITING = 3 };

int main(int argc, char** argv)
{
  oadrive::util::CameraCalibration myCal;
  bool found;
  int calibrationState = CAPTURING;
  cv::Mat view;
  cv::Mat image;
  cv::Mat input;
  if(argc > 2)
  {

    if(argc > 3)
    {
      std::string cameraCalPath = argv[3];
      myCal.readCameraCalFile(cameraCalPath);
    }
    std::string imagePath = argv[1];
    std::string calFilePath = argv[2];
    input = cv::imread(imagePath);
    /// Convert to grayscale
    cvtColor( input, input, CV_BGR2GRAY );

    /// Apply Histogram Equalization
    equalizeHist( input, image );
    cv::imshow("Input",image);
    cv::waitKey(1);
    if(calibrationState == CAPTURING ){

      found = myCal.autoCalPoints(image);
      if(found == true)
      {
        std::cout<<"pattern found"<<std::endl;
        calibrationState = CALIBRATED;
      }
      else
      {
        std::cout<<"No pattern found"<<std::endl;
        std::cout<<"Trying again with filters applied:"<<std::endl;
        cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                                     cv::Size( 3, 3 ) );
        cv::erode(image,image,element);
        cv::medianBlur(image,image,3);
        //cv::imshow("Eroded",image);
        found = myCal.autoCalPoints(image);
        if(found == true)
        {
          calibrationState = CALIBRATED;

          std::cout<<"\tFound after applying filters!"<<std::endl;
        } else {
          std::cout<<"\tStill no pattern found!"<<std::endl;
        }
      }
      view = myCal.drawCalPoints(image);
      cv::imshow("Found Points",view);
      cv::waitKey(1);
    }
    if(calibrationState == CALIBRATED)
    {
      view = myCal.transform(image);
      //            cv::imshow("BirdView",view);
      //            cv::waitKey(1);
      std::cout<<"calibriert"<<std::endl;
      std::cout<<"you can now adjust the image section then press a key and the file will be safed"<<std::endl;
      std::cout<<"Hit any Key to safe file!"<<std::endl;
      myCal.adjustPicture(image);
      myCal.estimateCameraPose();
      myCal.writeCalFile(calFilePath);

    }
    cv::waitKey(0);
  }
  else
  {
    std::cout<<"usage: Param 1: Image with circles; Param 2: Path for save Cal file ;opt. Param 3 Path to Camera Calfile"<<std::endl;
  }


}


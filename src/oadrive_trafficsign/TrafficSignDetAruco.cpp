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
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \author  Vitali Kaiser <vitali.kaiser@live.de>
 * \date    2016-01-19
 *
 */
//----------------------------------------------------------------------

#include "TrafficSignDetAruco.h"
#include <oadrive_core/ExtendedPose2d.h>
#include "trafficSignLogging.h"

using namespace oadrive::util;

using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive {
namespace trafficsign {

TrafficSignDetAruco::TrafficSignDetAruco(CoordinateConverter* converter, std::string configFolder)
  : birdViewPositionConverter(converter)
  , configurationFolder(configFolder)
{
  // first dictionary isn't loaded
  dictionaryLoaded = false;

  initialize();
}

TrafficSignDetAruco::~TrafficSignDetAruco() {

}

void TrafficSignDetAruco::initialize() {
  // get dictionary
  if(dictionary.fromFile(configurationFolder + "/roadsign.yml") == false) {
    LOGGING_ERROR(TrafficSignLogger, "[TrafficSignDetAruco] Dictionary file of road signs not found" << endl);
  }

  if(aruco::HighlyReliableMarkers::loadDictionary(dictionary) == false) {
    LOGGING_ERROR(TrafficSignLogger, "[TrafficSignDetAruco] loadDictionary function for markers failed (dictionary file not provided?)" << endl);
  }
  else {
    // dictionary is loaded, therefore marker detection is possible
    dictionaryLoaded = true;
  }

  // detector settings
  markerDetector.setMakerDetectorFunction(aruco::HighlyReliableMarkers::detect);
  markerDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
  markerDetector.setThresholdParams(THRESHOLD_PARAMETER_1, THRESHOLD_PARAMETER_2);
  markerDetector.setMinMaxSize(MIN_SIZE, MAX_SIZE);
  markerDetector.setWarpSize(WARP_SIZE);
}

void TrafficSignDetAruco::setCameraParameters(BirdViewConverter& birdViewConv) {
  cv::Mat distortionCoefficientsDouble = birdViewConv.getDistortionCoefficients();
  distortionCoefficientsDouble.convertTo(distortionCoefficients, CV_32F);

  cv::Mat cameraMatrixDouble = birdViewConv.getCameraMatrix();
  cameraMatrixDouble.convertTo(cameraMatrix, CV_32F);
}

/*boost::shared_ptr<TrafficSignDetAruco> TrafficSignDetAruco::getInstance() {
        if(!_instance) {
                boost::shared_ptr<TrafficSignDetAruco> temp(new TrafficSignDetAruco());
                //_instance(new TrafficSignDetAruco());
                _instance = temp;
        }

        return _instance;

}*/

void TrafficSignDetAruco::detectMarkers(cv::Mat& input, bool histogramEqualization /*= true*/) {
  // only perform detection if dictionary is loaded, otherwise aruco's detection crashes because of a division by zero
  if(dictionaryLoaded) {
    // next line not needed, it is performed in aruco's detect method
    //detectedMarkers.clear();

    cv::Mat equalizedImage(input.size(), input.type());
    cv::Mat* image;

    image = &input;

    // detect the marker
    markerDetector.detect(*image, detectedMarkers, cameraMatrix, distortionCoefficients, MARKER_SIZE);

    if(detectedMarkers.size() > 0) {
      // save current car pose
      oadrive::core::ExtendedPose2d positionCar( Environment::getInstance()->getCarPose() );

      // calculate position of markers in camera coordinate system and add every marker to environment
      for(size_t i = 0; i < detectedMarkers.size(); ++i) {
        // map traffic sign into range [0, 2*Pi)
        cv::Mat R;
        cv::Rodrigues(detectedMarkers[i].Rvec, R); // R is 3x3
        double r32 = R.at<float>(2,1);
        double r33 = R.at<float>(2,2);
        double trafficSignYaw = std::atan2(r32,r33);

        trafficSignYaw *= -1;

        int factor = trafficSignYaw / (2 * M_PI);
        trafficSignYaw -= factor * 2 * M_PI;
        if(trafficSignYaw < 0) {
          trafficSignYaw += 2 * M_PI;
        }

        // angle must be in range (Pi/2 to 3*Pi/2) - otherwise the car can't see the sign
        /*if(trafficSignYaw < M_PI_2) {
                                        trafficSignYaw += M_PI;
                                }
                                else if(trafficSignYaw > M_PI + M_PI_2) {
                                        trafficSignYaw -= M_PI;
                                }*/
        //std::cout << "yaw: " << trafficSignYaw << std::endl;
        oadrive::core::ExtendedPose2d positionMarkerCar(detectedMarkers[i].Tvec.at<float>(0) + CAMERA2CAR_X, detectedMarkers[i].Tvec.at<float>(2) + CAMERA2CAR_Y, trafficSignYaw);
        oadrive::core::ExtendedPose2d positionWorld( birdViewPositionConverter->car2World(positionCar, positionMarkerCar) );
        LOGGING_INFO(TrafficSignLogger, "traffic sign detected: " << detectedMarkers[i].id << ", x: " << detectedMarkers[i].Tvec.at<float>(0) + CAMERA2CAR_X << ", y: " << detectedMarkers[i].Tvec.at<float>(2) + CAMERA2CAR_Y << ", yaw: " << trafficSignYaw << endl);
        Environment::getInstance()->addTrafficSign(positionWorld, detectedMarkers[i].id);
      }
    }


      // histogram equalization
      cv::equalizeHist(input, equalizedImage);
      image = &equalizedImage;
    // detect the marker
    markerDetector.detect(*image, detectedMarkers, cameraMatrix, distortionCoefficients, MARKER_SIZE);

    if(detectedMarkers.size() > 0) {
      // save current car pose
      oadrive::core::ExtendedPose2d positionCar( Environment::getInstance()->getCarPose() );

      // calculate position of markers in camera coordinate system and add every marker to environment
      for(size_t i = 0; i < detectedMarkers.size(); ++i) {
        // map traffic sign into range [0, 2*Pi)
        cv::Mat R;
        cv::Rodrigues(detectedMarkers[i].Rvec, R); // R is 3x3
        double r32 = R.at<float>(2,1);
        double r33 = R.at<float>(2,2);
        double trafficSignYaw = std::atan2(r32,r33);

        trafficSignYaw *= -1;

        int factor = trafficSignYaw / (2 * M_PI);
        trafficSignYaw -= factor * 2 * M_PI;
        if(trafficSignYaw < 0) {
          trafficSignYaw += 2 * M_PI;
        }

        // angle must be in range (Pi/2 to 3*Pi/2) - otherwise the car can't see the sign
        /*if(trafficSignYaw < M_PI_2) {
                                        trafficSignYaw += M_PI;
                                }
                                else if(trafficSignYaw > M_PI + M_PI_2) {
                                        trafficSignYaw -= M_PI;
                                }*/
        //std::cout << "yaw: " << trafficSignYaw << std::endl;
        oadrive::core::ExtendedPose2d positionMarkerCar(detectedMarkers[i].Tvec.at<float>(0) + CAMERA2CAR_X, detectedMarkers[i].Tvec.at<float>(2) + CAMERA2CAR_Y, trafficSignYaw);
        oadrive::core::ExtendedPose2d positionWorld( birdViewPositionConverter->car2World(positionCar, positionMarkerCar) );
        LOGGING_INFO(TrafficSignLogger, "traffic sign detected: " << detectedMarkers[i].id << ", x: " << detectedMarkers[i].Tvec.at<float>(0) + CAMERA2CAR_X << ", y: " << detectedMarkers[i].Tvec.at<float>(2) + CAMERA2CAR_Y << ", yaw: " << trafficSignYaw << endl);
        Environment::getInstance()->addTrafficSign(positionWorld, detectedMarkers[i].id);
      }
    }
  }
}

void TrafficSignDetAruco::debugImage(cv::Mat& image) {
  for(size_t i = 0; i < detectedMarkers.size(); i++) {
    detectedMarkers[i].draw(image, cv::Scalar(0, 0, 255));

    LOGGING_DEBUG(TrafficSignLogger, "x " << detectedMarkers[i].Tvec.at<float>(0) <<
                  "y " << detectedMarkers[i].Tvec.at<float>(2) <<
                  "z " << detectedMarkers[i].Tvec.at<float>(1) << endl);

    cv::Mat cameraMatrixInvers;
    cv::invert(cameraMatrix, cameraMatrixInvers);
  }
}

} // namespace
} // namespace

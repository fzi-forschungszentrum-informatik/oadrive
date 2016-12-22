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

#ifndef PACKAGES_OADRIVE_SRC_OADRIVE_TRAFFICSIGN_TRAFFICSIGNDETARUCO_H_
#define PACKAGES_OADRIVE_SRC_OADRIVE_TRAFFICSIGN_TRAFFICSIGNDETARUCO_H_

#include "TrafficSignDetector.h"
#include <oadrive_util/BirdViewConverter.h>
#include <oadrive_util/CoordinateConverter.h>
#include <oadrive_world/Environment.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/imgcodecs.hpp>

#include <aruco/aruco.h>
#include <aruco/highlyreliablemarkers.h>

#define THRESHOLD_PARAMETER_1 7
#define THRESHOLD_PARAMETER_2 21
#define MIN_SIZE 0.005
#define MAX_SIZE 0.5
#define WARP_SIZE 80
#define MARKER_SIZE 0.12

// offset from camera origin to car origin
#define CAMERA2CAR_X -0.025
#define CAMERA2CAR_Y 0.295

using namespace oadrive::util;

namespace oadrive {
namespace trafficsign {

class TrafficSignDetAruco : TrafficSignDetector{
public:
  TrafficSignDetAruco(CoordinateConverter* converter, std::string configFolder);
  virtual ~TrafficSignDetAruco();

  /*! function sets cameraMatrix and distortionCoefficients which are obtained from bird view configuration
         *  CAUTION: needs to be set before findMarker function is performed!
         */
  void setCameraParameters(BirdViewConverter& birdViewConv);
  //	static boost::shared_ptr<TrafficSignDetAruco> getInstance();

  /*! detects markers in given image and adds detected markers to environment
         *  only works if valid dictionary file is laoded before, otherwise it does nothing
         *  \param img Should be gray image, otherwise it is converted into one
         *  \param histogramEqualization Defines if a histogram equalization is performed before detecting markers on image
         */
  void detectMarkers(cv::Mat& img, bool histogramEqualization = true);

  //! draws rectangles around detected markers in image and prints coordinates of marker (from camera coordinate system) to console
  void debugImage(cv::Mat& image);

private:
  //! bird view position converter
  CoordinateConverter* birdViewPositionConverter;

  //! config folder
  std::string configurationFolder;

  //! aruco detector
  aruco::MarkerDetector markerDetector;

  //! aruco detected markers
  std::vector<aruco::Marker> detectedMarkers;

  //! aruco dictionary of possible results
  aruco::Dictionary dictionary;

  //! camera matrix
  cv::Mat cameraMatrix;

  //! distortion coefficients
  cv::Mat distortionCoefficients;

  //! flag if dictionary is loaded (If dictionary is not loaded, marker detection can't be performed!)
  bool dictionaryLoaded;

  void initialize();
  static boost::shared_ptr<TrafficSignDetAruco> _instance;

};

}
}

#endif /* PACKAGES_OADRIVE_SRC_OADRIVE_TRAFFICSIGN_TRAFFICSIGNDETARUCO_H_ */

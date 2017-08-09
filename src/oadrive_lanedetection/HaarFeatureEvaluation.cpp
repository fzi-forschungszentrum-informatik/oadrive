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
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

#include "HaarFeatureEvaluation.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace oadrive::util;

namespace oadrive {
namespace lanedetection {


HaarFeatureEvaluation::HaarFeatureEvaluation( CoordinateConverter* converter ) : coordConverter( converter ) {
  x = converter->getImgSizeBirdView().width;
  y = converter->getImgSizeBirdView().height;

  HaarFilter haarFilter = HaarFilter(coordConverter);

  Mat image;

  /* center line 0.02
         * side line 0.03
         * cross road stop line 0.05
         */
  float width1 = 0.02 * coordConverter->getPixelsPerMeter();
  float width2 = 0.035 * coordConverter->getPixelsPerMeter();


  for(int i = 0; i < 360; i++) {
    std::cout << "Degree " << i << std::endl << std::flush;
    lines(width1, width2, i, image);

    haarFilter.setImage(image);
    haarFilter.calculateFeaturesPerLine( true, true );
    FeatureVector* features = haarFilter.getFeatures();

    Mat colorImage;
    cvtColor(image, colorImage, CV_GRAY2BGR);
    Mat debugFeatureImage = haarFilter.generateDebugImage();
    imshow("test", generateDebugFeatureImage(debugFeatureImage, colorImage));
    waitKey(0);

    for(FeatureVector::iterator it = features->begin(); it != features->end(); ++it) {
      std::cout << it->type << std::endl << std::flush;
    }
  }
}

HaarFeatureEvaluation::~HaarFeatureEvaluation() {

}

void HaarFeatureEvaluation::lines(int width1, int width2, int degrees, Mat& returnImage) {
  int imageX = x + x/2;
  int imageY = y + y/2;

  Mat image(imageY, imageX, CV_8U, Scalar(0,0,0));
  Rect r(x/4 + 1, y/4 + 1, x, y);

  rectangle(image, Point(imageX/2 - width1/2, 0), Point(imageX/2 + width1/2, imageY-1), Scalar(255, 255, 255), CV_FILLED);
  rectangle(image, Point(imageX/2 + x/4 - width2/2, 0), Point(imageX/2 + x/4 + width2/2, imageY-1), Scalar(255, 255, 255), CV_FILLED);

  Mat rotatedImage;

  rotate(image, degrees, rotatedImage);

  returnImage = rotatedImage(r);
}

void HaarFeatureEvaluation::rotate(cv::Mat& src, double angle, cv::Mat& dst) {
  Point2f rotationPoint((float)src.cols/2, (float)src.rows/2);
  Mat rotationMatrix = cv::getRotationMatrix2D(rotationPoint, angle, 1.0);

  warpAffine(src, dst, rotationMatrix, Size(src.cols, src.rows));
}

cv::Mat HaarFeatureEvaluation::generateDebugFeatureImage(cv::Mat& debugFeatureImage, cv::Mat& birdViewImage) {
  cv::Mat channels[4];
  cv::split( debugFeatureImage, channels );
  cv::Mat sum;
  cv::bitwise_or( birdViewImage, 0, sum, 255 - channels[3] );
  cv::Mat featuresRGB;
  cv::cvtColor( debugFeatureImage, featuresRGB, CV_BGRA2BGR );
  sum = sum + featuresRGB;

  return sum;
}

}
}

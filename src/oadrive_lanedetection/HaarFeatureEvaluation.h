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

#include "HaarFilter.h"
#include <oadrive_util/CoordinateConverter.h>
using namespace oadrive::util;
namespace oadrive{
namespace lanedetection{

class HaarFeatureEvaluation {
public:
  HaarFeatureEvaluation( CoordinateConverter* converter );
  ~HaarFeatureEvaluation();

private:
  int x;
  int y;
  CoordinateConverter* coordConverter;

  void lines(int width1, int width2, int rotateMaximum, cv::Mat& returnImage);
  void rotate(cv::Mat& src, double angle, cv::Mat& dst);
  cv::Mat generateDebugFeatureImage(cv::Mat& debugFeatureImage, cv::Mat& birdViewImage);
};

}
}

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
 * \date    2015-11-08
 *
 */
//----------------------------------------------------------------------


#ifndef LINE_H_
#define LINE_H_

#include "HaarFilter.h"

#define MINIMUM_POINTS_FOR_SPLINE_APPROXIMATION 50
#define ALLOWED_PIXEL_DIFFERENCE_TO_PREVIOUS_DETECTION 25
#define RECALCULATE_SPLINE_AFTER_VALID_DETECTIONS 5
#define BORDER_LEFT_RIGHT 15

namespace oadrive{
namespace lanedetection{

class Line {
public:
	Line();
	virtual ~Line();
	void setPoint(int y, int x);
	int getPointX(int y);
	int getLastValidDetectionY();
	void setDetected();
	bool isDetected();
	int getPredictionX(int y);
	bool isDetectionValid(int y, int lowerRangeX = -1, int upperRangeX = -1);

	// TODO: shouldn't be public!
	void calculateSpline(int x1, int y1, int x2, int y2, int x3, int y3, cv::Vec3f& coefficients);
	int getSplineValue(int y, cv::Vec3f& coefficients);

private:
	cv::Vec<int, IMAGE_Y> points;
	cv::Vec3f splineCoefficients;
	int splineLastCalculatedY;
	bool detected;
	int lastValidDetectionY;

	bool recalculateSpline(int y);
	int getSplinePredictionX(int y);
	void approximateMissedDetections(int nextValidDetection);
	void setLastValidDetectionY(int y);
};
}	// namespace
}	// namespace

#endif /* LINE_H_ */

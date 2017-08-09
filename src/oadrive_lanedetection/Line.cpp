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
 * \date    2015-11-08
 *
 */
//----------------------------------------------------------------------

#include "Line.h"

using namespace cv;
using namespace oadrive::lanedetection;

Line::Line() {
	lastValidDetectionY = IMAGE_Y;
	detected = false;
	points = Vec<int, IMAGE_Y>::all(0);

	splineCoefficients = Vec<float, 3>::all(0);
	splineLastCalculatedY = -1;
}

Line::~Line() {
	// TODO Auto-generated destructor stub
}

void Line::setPoint(int y, int x) {
	points(y) = x;
}

int Line::getPointX(int y) {
	return points(y);
}

void Line::setLastValidDetectionY(int y) {
	lastValidDetectionY = y;
}

int Line::getLastValidDetectionY() {
	return lastValidDetectionY;
}

void Line::setDetected() {
	detected = true;
}

bool Line::isDetected() {
	return detected;
}

int Line::getPredictionX(int y) {
	// if nothing was detected, no prediction is possible!
	if( !isDetected() ) {
		return -1;
	}

	// try to get spline prediction
	int x = getSplinePredictionX(y);

	// if spline prediction not possible use last valid detection as prediction
	if( x == -1 ) {
		x = getPointX(getLastValidDetectionY());
	}

	return x;
}

bool Line::isDetectionValid(int y, int lowerRangeX, int upperRangeX) {
	bool result = false;

	// no detection in row y
	if(getPointX(y) <= 0) {
		return false;
	}

	// use range check only if point never detected before
	if(!isDetected()) {
		// is valid range given
		if(lowerRangeX != -1 || upperRangeX != -1) {
			// check if point is in given range
			result = (getPointX(y) >= lowerRangeX && getPointX(y) <= upperRangeX);
		}
	}

	// use spline prediction if point detected before
	else {
		// is detection point close enough to predicted point?
		result = (abs(getPredictionX(y) - getPointX(y)) <= ALLOWED_PIXEL_DIFFERENCE_TO_PREVIOUS_DETECTION);
	}

	// if detection ran into border, don't use it
	if (getPointX(y) < BORDER_LEFT_RIGHT || getPointX(y) > IMAGE_X - BORDER_LEFT_RIGHT) {
		result = false;
	}

	if(result && y < getLastValidDetectionY()) {
		approximateMissedDetections(y);
		setLastValidDetectionY(y);
	}

	return result;
}

void Line::approximateMissedDetections(int nextValidDetectionY) {
	// are any detections missed?
	if(nextValidDetectionY + 1 < getLastValidDetectionY()) {
		for(int y = getLastValidDetectionY() - 1; y > nextValidDetectionY; y--) {
			setPoint(y, getPredictionX(y));
		}
	}
}

int Line::getSplinePredictionX(int y) {
	// recalculate spline, if requested position is too far away from pre-calculated spline
	// but only if the last valid detection isn't too far away
	if (recalculateSpline(y)) {
		Vec3f coefficients;
		int referencePoint = getLastValidDetectionY();
		int referencePoint2 = referencePoint + (IMAGE_Y - referencePoint) / 2;
		int referencePoint3 = IMAGE_Y - 1;
		calculateSpline(getPointX(referencePoint), referencePoint, getPointX(referencePoint2), referencePoint2, getPointX(referencePoint3), referencePoint3, coefficients);

		if(coefficients != Vec3f::zeros()) {
			splineCoefficients = coefficients;
			splineLastCalculatedY = y;
		}
	}

	if(splineLastCalculatedY != -1) {
		return getSplineValue(y, splineCoefficients);
	}
	return -1;
}

bool Line::recalculateSpline(int y) {
	// is last valid detection far enough down to calculate a spline?
	if(getLastValidDetectionY() > IMAGE_Y - MINIMUM_POINTS_FOR_SPLINE_APPROXIMATION) {
		return false;
	}

	// is last valid detection far enough away from last calculated position?
	if (abs(splineLastCalculatedY - getLastValidDetectionY()) > RECALCULATE_SPLINE_AFTER_VALID_DETECTIONS) {
		// is requested position far enough away to recalculate spline?
		if (abs(splineLastCalculatedY - y) > RECALCULATE_SPLINE_AFTER_VALID_DETECTIONS) {
			return true;
		}
	}

	return false;
}

void Line::calculateSpline(int x1, int y1, int x2, int y2, int x3, int y3, Vec3f& coefficients) {
	Mat A = (Mat_<float>(3,3) << pow(y1, 2), y1, 1,
								pow(y2, 2), y2, 1,
								pow(y3, 2), y3, 1);
	Vec3f B(x1, x2, x3);
	solve(A, B, coefficients);
}

int Line::getSplineValue(int y, Vec3f& coefficients) {
	return coefficients(0) * pow(y, 2) + coefficients(1) * y + coefficients(2);
}



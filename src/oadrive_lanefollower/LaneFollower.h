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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-24
 *
 * Center-Piece for Oadrive, representtion of the current world.
 *
 */
//----------------------------------------------------------------------

#ifndef KACADU_LANEFOLLOWER_H
#define KACADU_LANEFOLLOWER_H

#include <oadrive_control/LateralController.h>
#include <oadrive_core/Types.h>
#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/Interpolator.h>
#include <oadrive_util/CoordinateConverter.h>

#include <opencv2/imgproc/imgproc.hpp>


#define HAARFILTER
//#define NEURALNETS

#ifdef HAARFILTER
	//#include "HaarLaneTracker/HaarFilter.h"
	#include <oadrive_lanedetection/HaarFilter.h>
#else
	#include "NNLaneTracker/NNPredictor.h"
#endif

class LaneFollower
{
	public:
		LaneFollower();
		~LaneFollower() {};
		
		void setImage( cv::Mat image );
		void setPose( float x, float y, float yaw );

		cv::Mat getResultImage();

		void createTestTrajectory();
		float steer();

		oadrive::core::Trajectory2d getTrajectory() { return mTrajectory; }

		void setConfigFile( std::string file );

	private:

		oadrive::core::ExtendedPose2d mPose;

		oadrive::core::Trajectory2d mTrajectory;
		oadrive::control::LateralController mLateralController;

#ifdef HAARFILTER
		oadrive::lanedetection::HaarFilter mHaarFilter;
#else
		NNPredictor mNNPredictor;
#endif

		oadrive::util::CoordinateConverter mCoordConverter;
};



#endif

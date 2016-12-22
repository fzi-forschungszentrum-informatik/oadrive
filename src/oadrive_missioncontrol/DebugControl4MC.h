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
 * \author  Vitali Kaiser <vitali.kaiser@live.de>
 * \date    2015-12-16
 *
 */
//----------------------------------------------------------------------
/*
#ifndef MISSIONCONTROL_DEBUGCONTROL4MC_H
#define MISSIONCONTROL_DEBUGCONTROL4MC_H

#include "IControl4MC.h"
#include "../oadrive_control/IDriverModule.h"
#include "../oadrive_lanedetection/IStreetPatcher.h"
#include "../oadrive_world/IEnvironment.h"
#include "../oadrive_world/ITrajectoryFactory.h"
#include <iostream>

namespace oadrive{
namespace missioncontrol{

//Outputs everything to console
class DebugControl4MC : public IControl4MC {
	public:
	void setJuryState(stateCar state, int manID);
	stateCar lastState;
	int lastManID;

	void setLights(enumLight light, bool on);
	enumLight lastLight;
	bool lastOn;

	oadrive::util::Timer* getTimer();

	oadrive::util::Timer timer;

};

class DummyDriverControl: public IDriverModule {
public:
	void drive();
	void halt();
	void setTargetSpeed(float speed);
	float lastSpeed;
};

class DummyStreetPatcher: public IStreetPatcher {
public:
	void reset();
	oadrive::world::ParkingType getParkingSpaceDirection( cv::Mat &image ) {
		return PARKING_TYPE_UNKNOWN;
	}
};

class DummyEnvironment: public IEnvironment {

};

class DummyTrajectoryFactory: public ITrajectoryFactory {
	enumTrajectory lastMan;
	void setFixedTrajectory( enumTrajectory trajName,
			oadrive::world::PatchPtr patch = oadrive::world::PatchPtr() );
	void setGenerateFromPatches() {};
	bool generateFromPatches();
};


}	// namespace
}	// namespace

#endif //MISSIONCONTROL_DEBUGCONTROL4MC_H
/*

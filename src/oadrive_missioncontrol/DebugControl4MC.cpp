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
#include "DebugControl4MC.h"

#include "oadrive_missioncontrol/mcLogging.h"
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive{
namespace missioncontrol{

void DebugControl4MC::setJuryState (stateCar state, int manID) {
        lastState = state;
        lastManID = manID;
    //TODO State als Beschreibung
    LOGGING_INFO( mcLogger, "DummyControl4MC State: " << state << "with manID "<< manID << endl );

}

oadrive::util::Timer* DebugControl4MC::getTimer() {
        return &timer;
}

void DebugControl4MC::setLights(enumLight light, bool on) {
        lastLight = light;
        lastOn = on;
        if(on) {
                LOGGING_INFO( mcLogger, "DummyControl4MC turning on light: " << light << endl);
        } else {
                LOGGING_INFO( mcLogger, "DummyControl4MC turning off light: " << light << endl);
        }
}

void DummyDriverControl::drive() {
        LOGGING_INFO( mcLogger, "DummyDriverControl drive() method." << endl);

}

void DummyDriverControl::halt() {
        LOGGING_INFO( mcLogger, "DummyDriverControl halt() method." << endl);
}

void DummyDriverControl::setTargetSpeed(float speed) {
        lastSpeed = speed;
        LOGGING_INFO( mcLogger, "DummyDriverControl setTargetSpeed(" << speed <<")." << endl);
}

void DummyStreetPatcher::reset() {
        LOGGING_INFO( mcLogger, "DummyStreetPatcher reset() method." << endl);
}

bool DummyTrajectoryFactory::generateFromPatches() {
        LOGGING_INFO( mcLogger, "DummyTrajectoryFactory generateFromPatches() method." << endl);
        return true;
}

void DummyTrajectoryFactory::setFixedTrajectory(enumTrajectory man,
                oadrive::world::PatchPtr patch ) {
        lastMan = man;
        LOGGING_INFO( mcLogger, "DummyTrajectoryFactory setFixedTrajectory(" << man <<")." << endl);
}

}	// namespace
}	// namespace

*/

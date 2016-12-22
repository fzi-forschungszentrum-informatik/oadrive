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
 * \date    2016-01-16
 *
 */
//----------------------------------------------------------------------

#ifndef PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MISSIONCONTROLENUMS_H_
#define PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MISSIONCONTROLENUMS_H_



/**
* Left				An der nächsten Kreuzung links abbiegen
* Right				An der nächsten Kreuzung rechts abbiegen
* Straight			An der nächsten Kreuzung geradeaus fahren
* parallel_parking	Am nächsten Parkplatz in Fahrtrichtung rechts längs einparken
* cross_parking		Am nächsten Parkplatz in Fahrtrichtung rechts quer einparken
* pull_out_left		Parkplatz nach links verlassen (vom Parkplatz senkrecht in Richtung Straße blickend, nur in Verbindung mit Querparken)
* pull_out_right	Parkplatz nach rechts verlassen (vom Parkplatz senkrecht in Richtung Straße blickend)
* finished			!! Selbst definiert. Der nächste Job ist quasi stehen bleiben.
 */
enum enumManeuver {MANEUVER_LEFT, MANEUVER_RIGHT, MANEUVER_STRAIGHT, MANEUVER_PARKING_PARALLEL,
	MANEUVER_PARKING_CROSS, MANEUVER_PULLOUT_LEFT, MANEUVER_PULLOUT_RIGHT, MANEUVER_FINISHED};

enum enumLight { HEAD_LIGHT, REVERSE_LIGHT, BRAKE_LIGHT, BLINK_RIGHT_LIGHT, BLINK_LEFT_LIGHT,
	HAZARD_LIGHT };

#endif /* PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MISSIONCONTROLENUMS_H_ */

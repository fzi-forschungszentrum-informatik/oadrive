// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE in the top
// directory of the source code.
//
// © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Fabian Dürr
 * \date    2017-9-26
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_LANEDETECTION_ROADPERCEPTIONCONFIG_H
#define OADRIVE_LANEDETECTION_ROADPERCEPTIONCONFIG_H


/****************************************************/
/************** General Properties ******************/
/****************************************************/

const float ROAD_WIDTH = 0.96f;                    // [m]
const float HALF_ROAD_WIDTH = ROAD_WIDTH * 0.5f;  // [m]
const float LANE_WIDTH = HALF_ROAD_WIDTH;         // [m]

const float PATCH_LENGTH_STRAIGHT = 0.25f;        // [m]
const float PATCH_LENGTH_INTERSECTION = 0.96;     // [m]
const float PATCH_LENGTH_PARKINGSPOT = 0.85f;     // [m]

const float PATCH_WIDTH_STRAIGHT = ROAD_WIDTH;    // [m]
const float PATCH_WIDTH_INTERSECTION = 0.96f;      // [m]
const float PATCH_WIDTH_PARKINGSPOT = 0.45f;     // [m]


/****************************************************/
/***************** General Patcher ******************/
/****************************************************/
const unsigned int MAX_VOTING_SCORE = 1000;
const float DISCOUNTFACTOR = 1.0;

/****************************************************/
/****************** StreetPatcher *******************/
/****************************************************/
const float STREETPATCH_MIN_VOTE_COMBINED = 50;
const float STREETPATCH_MIN_VOTE_FEATURE_ONLY = 80;
const float CROSSPATCH_MIN_VOTE_FEATURE_ONLY = 30;
const float CROSSPATCH_MIN_OVERLAP = 0.9;

/****************************************************/
/*************** IntersectionPatcher ****************/
/****************************************************/
const float INTERSECTION_PATCH_BASED_MIN_VOTE = 30;
const float INTERSECTION_TRAFFIC_BASED_MIN_VOTE = 30;
const float INTERSECTION_MIN_OVERLAP = 0.6;



#endif //OADRIVE_LANEDETECTION_ROADPERCEPTIONCONFIG_H

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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-17
 *
 */
//----------------------------------------------------------------------

#include "EventRegion.h"
#include <iostream>
#include <limits>

using namespace oadrive::core;

namespace oadrive{
namespace world{

EventRegion::EventRegion( EventRegionType type, const oadrive::core::ExtendedPose2d &pose,
                          float width, float height )
  : EnvObject( ExtendedPose2d( 0,0,0 ), width, height )
  , mLocalPose( pose )
{
  mEventRegionType = type;
  mIsCarInside = false;
  mToDelete = false;
}

EventRegion::~EventRegion(){}

}	// namespace
}	// namespace

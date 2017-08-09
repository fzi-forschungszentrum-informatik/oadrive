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
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-19
 *
 */
//----------------------------------------------------------------------

#include "TrafficSign.h"

using namespace oadrive::core;

namespace oadrive{
namespace world{

TrafficSign::TrafficSign( int type, double x, double y )
  : EnvObject( ExtendedPose2d( x,y,0 ), 0, 0 )
{
  mTrafficSignType = type;
}

TrafficSign::TrafficSign( int type, const ExtendedPose2d &pose )
  : EnvObject( pose, 0, 0 )
{
  mTrafficSignType = type;
}

TrafficSign::~TrafficSign(){}



}	// namespace
}	// namespace

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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-19
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#include "TrafficSign.h"

using namespace oadrive::core;

namespace oadrive {
namespace world {

TrafficSign::TrafficSign(TrafficSignType type, double x, double y)
    : EnvObject(ExtendedPose2d(x, y, 0), 0, 0) {
  mTrafficSignType = type;
}

TrafficSign::TrafficSign(TrafficSignType type, const ExtendedPose2d &pose) : EnvObject(pose, 0, 0) {
  mTrafficSignType = type;
}

TrafficSign::TrafficSign() :
        EnvObject(ExtendedPose2d(0.f, 0.f, 0.f), 0.f, 0.f),
        mTrafficSignType(INVALID)
{
}

TrafficSign::~TrafficSign() {}

bool TrafficSign::isIntersectionTrafficSign() const {
    switch (this->getType())
    {
      case world::GIVE_WAY:
      case world::STOP_AND_GIVE_WAY:
      case world::UNMARKED_INTERSECTION:
      case world::HAVE_WAY:
        return true;
      default:
        return false;
    }
  }

}  // namespace world
}  // namespace oadrive

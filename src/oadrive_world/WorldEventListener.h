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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-14
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_WORLDEVENTLISTENER_H
#define OADRIVE_WORLD_WORLDEVENTLISTENER_H

#include <oadrive_world/Patch.h>
#include <oadrive_world/EventRegion.h>

namespace oadrive{
namespace world{

class WorldEventListener{
public:
  WorldEventListener() {};
  virtual ~WorldEventListener() {};

  /*! Called when trajectory could be generated (i.e. patches were found). */
  virtual void eventReadyToGeneratePatchTrajectory() = 0;

  /*! Called when an empty trajectory has been passed to the environment: */
  virtual void eventTrajectoryEmpty() = 0;

  virtual void eventRegionTriggered( EventRegionPtr evRegion, EventType evType ) = 0;
public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}	// namespace
}	// namespace

#endif  // OADRIVE_WORLD_WORLDEVENTLISTENER_H

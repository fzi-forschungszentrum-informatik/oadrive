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
 * \author  Kolja Esders <kolja.esders@gmail.com>
 * \date		2017-09-10
 *
 *  This file defines generic events.
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_EVENT_H
#define OADRIVE_CORE_EVENT_H

#include <string>

namespace oadrive {
namespace core {

enum EventType {
  EVENT_REGION_ENTERED, //! (1st) Fired when entering an event region.
  EVENT_REGION_LEFT,    //! (2nd) Fired when leaving an event region.
  OBJECT_CLOSE,         //! (1st) Fired when closing in on an object.
  OBJECT_DISTANT        //! (2nd) Fired when moving away from close object.
};

struct Event {
  EventType type;
};

}  // namespace core
}  // namespace oadrive

#endif

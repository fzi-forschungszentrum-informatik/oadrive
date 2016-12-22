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
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \date    2016-01-17
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_CONTROL_LOGGING_H
#define OADRIVE_CONTROL_LOGGING_H
#include "icl_core_logging/Logging.h"
namespace oadrive {
namespace control {
DECLARE_LOG_STREAM(controlLogger);
DECLARE_LOG_STREAM(latLogger);
}
}

#endif // OADRIVE_CONTROL_LOGGING_H

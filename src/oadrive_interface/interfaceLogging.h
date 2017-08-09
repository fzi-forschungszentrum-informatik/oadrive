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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-12-01
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_INTERFACE_INTERFACELOGGING_H
#define OADRIVE_INTERFACE_INTERFACELOGGING_H
#include "icl_core_logging/Logging.h"
namespace oadrive{
namespace interface{
DECLARE_LOG_STREAM(interfaceLogger);
DECLARE_LOG_STREAM(USDataLogger);
}
}

#endif // LOGGING_H

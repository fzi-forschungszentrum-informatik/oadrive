// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the Open Autonomous Driving Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2015 FZI Forschungszentrum Informatik, Karlsruhe, Germany

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Sebastian Klemm
 * \date    2015-02-09
 *
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_LOGGING_H_INCLUDED
#define OADRIVE_LOGGING_H_INCLUDED

#include <oadrive_core/ImportExport.h>
#include <icl_core_logging/Logging.h>

namespace oadrive {

DECLARE_LOG_STREAM_IMPORT_EXPORT(OADriveLogger, OADRIVE_IMPORT_EXPORT)

} // namespace

#define OADRIVE_ERROR(message) LOGGING_ERROR(oadrive::OADriveLogger, message << icl_core::logging::endl);
#define OADRIVE_WARNING(message) LOGGING_WARNING(oadrive::OADriveLogger, message << icl_core::logging::endl);
#define OADRIVE_INFO(message) LOGGING_INFO(oadrive::OADriveLogger, message << icl_core::logging::endl);
#define OADRIVE_DEBUG(message) LOGGING_DEBUG(oadrive::OADriveLogger, message << icl_core::logging::endl);
#define OADRIVE_TRACE(message) LOGGING_TRACE(oadrive::OADriveLogger, message << icl_core::logging::endl);

#endif

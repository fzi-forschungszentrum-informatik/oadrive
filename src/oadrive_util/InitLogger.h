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
 * \author  Peter Zimmer <peter-zimmer@gmx.net>
 * \date    2016-01-17
 *
 */
//----------------------------------------------------------------------

#ifndef LOGGER_H
#define LOGGER_H
#include <icl_core_logging/Logging.h>
namespace oadrive {
namespace util {
using icl_core::logging::endl;
using icl_core::logging::flush;

class InitLogger
{
public:
  InitLogger(std::string filename);
  ~InitLogger();
};
}
}

#endif // LOGGER_H
/*
  Debug Levels
  Trace (LOGGING_TRACE)
  Debug (LOGGING_DEBUG)
  Info (LOGGING_INFO)
  Warning (LOGGING_WARNING)
  Error (LOGGING_ERROR)
 */

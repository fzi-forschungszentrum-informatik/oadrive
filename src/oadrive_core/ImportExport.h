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
#ifndef OADRIVE_IMPORT_EXPORT_H_INCLUDED
#define OADRIVE_IMPORT_EXPORT_H_INCLUDED

#if defined(_SYSTEM_WIN32_) && !defined(_IC_STATIC_)
#  pragma warning ( disable : 4251 )

# if defined OADRIVE_EXPORT_SYMBOLS
#  define OADRIVE_IMPORT_EXPORT __declspec(dllexport)
# else
#  define OADRIVE_IMPORT_EXPORT __declspec(dllimport)
# endif

#elif defined(__GNUC__) && (__GNUC__ > 3) && !defined(_IC_STATIC_)

# define OADRIVE_IMPORT_EXPORT __attribute__ ((visibility("default")))

#else

# define OADRIVE_IMPORT_EXPORT

#endif

#endif

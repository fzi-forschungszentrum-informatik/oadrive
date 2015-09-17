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
 * \author  Sebastian Klemm <klemm@fzi.de>
 * \date    2014-03-07
 *
 *  This file holds some utility functions
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_UTILITY_H_INCLUDED
#define OADRIVE_CORE_UTILITY_H_INCLUDED


# include <iostream>
# include <sstream>

namespace oadrive {
namespace core {

template <typename U, typename T> U lexical_cast(const T& t)
{
  std::stringstream ss;
  ss << t;
  U u;
  ss >> u;
  return u;
}

template <typename T> std::string to_string(const T& t)
{
  return lexical_cast<std::string>(t);
}

} // end of ns
} // end of ns

#endif

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
 *  This file holds some typedefs as shortcut to frequently used types
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_TYPES_H_INCLUDED
#define OADRIVE_CORE_TYPES_H_INCLUDED

#include <Eigen/Dense>

namespace oadrive {
namespace core {

typedef Eigen::Vector2d Position2d;
typedef Eigen::Quaterniond Quaternion;

} // end of ns
} // end of ns

#endif

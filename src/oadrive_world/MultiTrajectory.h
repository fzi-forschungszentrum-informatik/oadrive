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
 * \date    2016-02-24
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_CONTROL_MULTITRAJECTORY_H
#define OADRIVE_CONTROL_MULTITRAJECTORY_H

namespace oadrive {
namespace world {

//! \todo FIX THESE according to Eigen Guidelines!
struct MultiTrajectory
{
  std::vector<oadrive::core::Trajectory2d> trajectories;

  MultiTrajectory() : trajectories(0) {}

  // Is this enough for Eigen Guidelines fix?
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}   // namespace
}   // namespace

#endif

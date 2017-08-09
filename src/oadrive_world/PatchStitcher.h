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
 * \date    2015-12-24
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_PATCHSTITCHER_H
#define OADRIVE_WORLD_PATCHSTITCHER_H

#include "Patch.h"
#include "PatchStitcher.h"

namespace oadrive{
namespace world{

class PatchStitcher
{
public:
  PatchStitcher();
  virtual ~PatchStitcher() {};
  bool addStreetPatch( PatchPtr streetPatch );

private:

  /*! Attempt to merge the new patch into the list of patches.
                 * The new patch is merged if another patch is found which is similar (position and type).
                 * \return true if new patch was "absorbed" (merged) into the list, false otherwise. */
  bool mergeIntoList( PatchPtr newPatch );
  PatchPtr findReferencePatchFor( PatchPtr patch );
  /*! Returns true if the patch overlaps with any CROSS_SECTIONs. */
  bool overlapsCrossSection( PatchPtr patch );
  
  /* Find out if the given Patch is too close to a CROSS_SECTION: */
  bool tooCloseToCrossSection( PatchPtr patch, float dist );

  float mMaximumAngle;
  float mMinimumDistBetweenCrossings;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}	// namespace
}	// namespace

#endif  // OADRIVE_WORLD_PATCHSTITCHER_H

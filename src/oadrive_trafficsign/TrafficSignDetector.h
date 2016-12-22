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
 * \author  Jan-Markus Gomer <uecxl@student.kit.edu>
 * \author  Vitali Kaiser <vitali.kaiser@live.de>
 * \date    2016-01-19
 *
 */
//----------------------------------------------------------------------

#ifndef PACKAGES_OADRIVE_SRC_OADRIVE_TRAFFICSIGN_TRAFFICSIGNDETECTOR_H_
#define PACKAGES_OADRIVE_SRC_OADRIVE_TRAFFICSIGN_TRAFFICSIGNDETECTOR_H_

#include <boost/shared_ptr.hpp>

namespace oadrive {
namespace trafficsign {

//! Traffic Sign Detector Interface, so we change the implementation later on
class TrafficSignDetector {
public:
  TrafficSignDetector(){};
  virtual ~TrafficSignDetector(){};


};

}
}

#endif /* PACKAGES_OADRIVE_SRC_OADRIVE_TRAFFICSIGN_TRAFFICSIGNDETECTOR_H_ */

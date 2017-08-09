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
 * \author  Vitali Kaiser <vitali.kaiser@live.de>
 * \date    2016-02-07
 *
 */
//----------------------------------------------------------------------

#ifndef PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_IMC2MAN_H_
#define PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_IMC2MAN_H_

#include "juryEnums.h"
#include <boost/shared_ptr.hpp>

//! \brief Interface for maneuver list to manipulate MissionControl :)
class IMC2Man {
public:
  virtual ~IMC2Man() {};
  typedef boost::shared_ptr<IMC2Man> Ptr;
  typedef boost::shared_ptr<const IMC2Man> ConstPtr;


  virtual void eventParcourFinished() = 0;
  virtual void eventSearchParking() = 0;
  virtual void setJuryState( stateCar state, int manID ) = 0;


};




#endif /* PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_IMC2MAN_H_ */

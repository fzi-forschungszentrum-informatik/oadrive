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
 * \date    2015-12-16
 *
 */
//----------------------------------------------------------------------

#ifndef ICMAKER_ICONTROL4MC_H
#define ICMAKER_ICONTROL4MC_H

#include "juryEnums.h"
#include "MissionControlEnums.h"
#include <oadrive_util/Timer.h>
#include <oadrive_world/TrajectoryDatabase.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace oadrive{
namespace missioncontrol{

class IControl4MC {
public:

  typedef boost::shared_ptr<IControl4MC> Ptr;
  typedef boost::shared_ptr<const IControl4MC> ConstPtr;


  virtual void setJuryState( stateCar state, int manEntry) = 0;

  virtual void setLights( enumLight light, bool on ) = 0;

  virtual oadrive::util::Timer::Ptr getTimer() = 0;

  virtual cv::Mat getLastBirdViewImage() = 0;

  virtual void reset() = 0;

  virtual ~IControl4MC() {}
};

}	// namespace
}	// namespace


#endif //ICMAKER_ICONTROL4MC_H

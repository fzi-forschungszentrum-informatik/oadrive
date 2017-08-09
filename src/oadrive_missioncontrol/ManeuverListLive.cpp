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
 * \date    2016-02-27
 *
 */
//----------------------------------------------------------------------

#include "ManeuverListLive.h"
#include <string>
#include <iostream>
#include <oadrive_util/Broker.h>

//next 3 lines are for logging
#include "oadrive_missioncontrol/mcLogging.h"
using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace oadrive::util;

namespace oadrive
{
  namespace missioncontrol
  {
    using namespace boost::assign;
    static const std::string channelName = "oadrive/manlist";


    map<string, enumManeuver> ManeuverListLive::maneuverMap = map_list_of ("finished", MANEUVER_FINISHED) ("left", MANEUVER_LEFT) ("right", MANEUVER_RIGHT) ("straight", MANEUVER_STRAIGHT) ("parallel_parking", MANEUVER_PARKING_PARALLEL) ("cross_parking", MANEUVER_PARKING_CROSS) ("pull_out_left", MANEUVER_PULLOUT_LEFT) ("pull_out_right", MANEUVER_PULLOUT_RIGHT);


    ManeuverListLive::ManeuverListLive ()
    {
      m_currentMan = MANEUVER_RIGHT;
      m_prevMan = MANEUVER_RIGHT;
      m_manID = 0;
      m_isFin = false;
      mc = IMC2Man::Ptr();

      LOGGING_INFO( mcLogger, "ManeuverListLive constructed." << endl );

      // Initialize the broker:
      Broker::getInstance();
    }

    void ManeuverListLive::onMessage(const std::vector<char> &buf) {
      std::string msg(buf.begin(), buf.end());
    }


    void ManeuverListLive::increase() {
      m_manID++;
    }

    std::string ManeuverListLive::toString() {
      return "";
    }
    unsigned int ManeuverListLive::getCurrentAbsManeuverID() {
      return m_manID;
    }

    enumManeuver ManeuverListLive::getCurrentManeuver() {
      // When asked for a new manevuer, simply retrieve the last received maneuver:
      m_currentMan = Broker::getInstance()->getLastReceivedManeuver();
      LOGGING_INFO( mcLogger, "Current maneuver in ManeuverListLive: " << m_currentMan << endl );
      return m_currentMan;
    }

    enumManeuver ManeuverListLive::getPreviosManeuver() {
      return m_prevMan;
    }
    bool ManeuverListLive::isFinished() {
      return m_isFin;
    }
    bool ManeuverListLive::setManeuverId(int maneuverEntryID) {
      //ignore that... we dont have ids.....
      return true;
    }
    bool ManeuverListLive::isDummy() {
      // we are the real thing ;)
      return false;
    }

  } /* namespace missioncontrol */
} /* namespace oadrive */

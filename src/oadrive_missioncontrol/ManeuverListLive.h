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
 * \author  Vitali Kaiser <vitali.kaiser@live.de>
 * \date    2016-02-27
 *
 */
//----------------------------------------------------------------------

#ifndef PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MANEUVERLISTLIVE_H_
#define PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MANEUVERLISTLIVE_H_

#include "IManeuverList.h"
#include "IMC2Man.h"
#include <vector>
#include <boost/assign/list_of.hpp> // for 'map_list_of()'
#include <boost/assert.hpp>
#include <map>

namespace oadrive
{
  namespace missioncontrol
  {

    using namespace std;

    class ManeuverListLive : public IManeuverList
    {
    public:
      ManeuverListLive ();
      void increase();
      std::string toString();
      unsigned int getCurrentAbsManeuverID();
      enumManeuver getCurrentManeuver();
      enumManeuver getPreviosManeuver();
      bool isFinished();
      bool setManeuverId(int maneuverEntryID);
      bool isDummy();
      IMC2Man* mc;
      void onMessage(const std::vector<char> &buf);
      bool isLive() { return true; }

    private:
      enumManeuver m_currentMan;
      enumManeuver m_prevMan;
      unsigned int m_manID;
      bool m_isFin;
      static map<string, enumManeuver> maneuverMap;
    };

  } /* namespace missioncontrol */
} /* namespace oadrive */

#endif /* PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MANEUVERLISTLIVE_H_ */

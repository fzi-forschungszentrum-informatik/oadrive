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
 * \date    2016-01-08
 *
 */
//----------------------------------------------------------------------


#ifndef PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MANEUVERLIST_H_
#define PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MANEUVERLIST_H_

#include <vector>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <map>
#include "boost/assign.hpp"
#include <boost/smart_ptr.hpp>
#include "MissionControlEnums.h"
#include "IMC2Man.h"
#include "IManeuverList.h"

namespace oadrive{
namespace missioncontrol{

using namespace std;
using namespace boost::assign;


class ManeuverList : public IManeuverList {
  class AADC_Sector;

  class AADC_Maneuver {
  public:
    int id;
    enumManeuver action;
    boost::shared_ptr<AADC_Sector> parent;
    AADC_Maneuver(int id = 0, enumManeuver action = MANEUVER_FINISHED, boost::shared_ptr<AADC_Sector> parent = boost::shared_ptr<AADC_Sector>()) {
      this->id = id;
      this->action = action;
      this->parent = parent;
    }
  };

  class AADC_Sector {
  public:
    int id;
    vector<boost::shared_ptr<AADC_Maneuver> > maneuvers;
    AADC_Sector(int id = 0) {
      this->id = id;
      this->maneuvers = vector<boost::shared_ptr<AADC_Maneuver> > ();
    }
  };

public:
  //! parses a given String to a Maneuver list
  static boost::shared_ptr<IManeuverList> parse(string input, IMC2Man::Ptr mc);

  //! cant make backCalls! Please dont use this anymore :)
  static boost::shared_ptr<IManeuverList> parse(string input);

  //static boost::shared_ptr<ManeuverList> parse(string input);

  /**!
         * Gives Maneuver back starting by the first Element in the whole list independent of current maneuver.
         * shift has to be positive. If its out of range it returns the first or the last possible Maneuver
     */
  enumManeuver getAbsoluteManeuver(int shift);

  /*! Return ID of the current maneuver (absolute!) */
  unsigned int getCurrentAbsManeuverID() { return currentDaManeuver; }

  /**!
         * Gives Maneuver back relative to current Maneuver.
         * shift 0  returns current back. -1 return the previous Maneuver. 1 returns next Maneuver.
         */
  enumManeuver getRelativeManeuver(int shift);

  //! returns the current Maneuver (the one we should execute next)
  enumManeuver getCurrentManeuver();
  //! returns the Maneuver after the current Maneuver
  enumManeuver getNextManeuver();
  //! returns  the Maneuver before the current Maneuver
  enumManeuver getPreviosManeuver();

  //! Returns true if we're in the final, "finished" state:
  bool isFinished();

  //! returns the current sector number (first sector equals 0)
  unsigned int getCurrentSector();

  /**!
         * increases the current to the next Maneuver.
         * if its already the last, we dont increase any more.
         */
  void increase();

  //! Returns the description defined in the xml
  string getDescription();

  //! set the current maneuver to the given id. If id cant be found it returns false, in all other cases true;
  bool setManeuverId(int maneuverEntryID);

  //! Dummy Constructor for using if no ManeuverList is available.
  static boost::shared_ptr<IManeuverList> getDummy();

  //! Debug output of the maneuver list:
  std::string toString();

  //! Get a string representation of the enum maneuver entry:
  static std::string maneuverAsString( enumManeuver man );

  bool isDummy();

private:
  unsigned int currentSectorPos;
  unsigned int currentManeuverPos;
  unsigned int currentDaManeuver;
  bool dummy;
  string desc;
  vector<boost::shared_ptr<AADC_Sector> > sectors;
  vector<boost::shared_ptr<AADC_Maneuver> > daManeuver; //direct access to the maneuvers
  //! Maps a Maneuver as string to the correct enum!
  static map<string, enumManeuver> maneuverMap;
  IMC2Man::Ptr mc;
  ManeuverList();

};

typedef boost::shared_ptr<IManeuverList> ManeuverListPtr;

}	// namespace
}	// namespace

#endif /* PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_MANEUVERLIST_H_ */

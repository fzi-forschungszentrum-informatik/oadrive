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
 * \date    2016-01-08
 *
 */
//----------------------------------------------------------------------

#include "ManeuverList.h"
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <exception>
#include <algorithm>
#include <iostream>
#include "mcLogging.h"
#include "ManeuverListLive.h"

using icl_core::logging::endl;
using icl_core::logging::flush;

using namespace boost;

namespace oadrive{
namespace missioncontrol{

map<string, enumManeuver> ManeuverList::maneuverMap = map_list_of ("left", MANEUVER_LEFT) ("right", MANEUVER_RIGHT) ("straight", MANEUVER_STRAIGHT) ("parallel_parking", MANEUVER_PARKING_PARALLEL) ("cross_parking", MANEUVER_PARKING_CROSS) ("pull_out_left", MANEUVER_PULLOUT_LEFT) ("pull_out_right", MANEUVER_PULLOUT_RIGHT);

ManeuverList::ManeuverList()
{
  this->currentManeuverPos = 0;
  this->currentSectorPos = 0;
  this->currentDaManeuver = 0;
  dummy = true;
}

boost::shared_ptr<IManeuverList> ManeuverList::getDummy() {

  boost::shared_ptr<ManeuverList> result(new ManeuverList());
  //Always add a last "Finished Sector"
  shared_ptr<AADC_Sector> finSec(new AADC_Sector(INT32_MAX));
  shared_ptr<AADC_Maneuver> finMan(new AADC_Maneuver(INT32_MAX, MANEUVER_FINISHED,finSec));

  result->daManeuver.push_back(finMan);
  finSec->maneuvers.push_back(finMan);
  result->sectors.push_back(finSec);

  return static_pointer_cast<IManeuverList>(result);
}

boost::shared_ptr<IManeuverList> ManeuverList::parse(string input) {
  return parse(input,0);
}

//! parses the maneuver list from an xml string. Throws an exception if input string can not be parsed.
boost::shared_ptr<IManeuverList> ManeuverList::parse(string input, IMC2Man* mc) {
  using boost::lexical_cast; //for parsing string to int
  using boost::bad_lexical_cast;


  // Create an empty property tree object
  using boost::property_tree::ptree;
  ptree pt;
  // Read the XML config string into the property tree. Catch any exception
  stringstream ss; ss << input;
  read_xml(ss, pt);

  boost::shared_ptr<ManeuverList> result(new ManeuverList());

  result->mc = mc;

  try {
    result->desc = pt.get_child("AADC-Maneuver-List.<xmlattr>.description").data();
  } catch (std::exception& e) {

    LOGGING_WARNING( mcLogger, "WARNING: description not defined!" << e.what() << endl );
  }

  BOOST_FOREACH(const ptree::value_type & v, pt.get_child("AADC-Maneuver-List")){

    if(v.first == "AADC-Sector") {


      int id = 0;
      // Try to get id... works even with out but gives a warning
      try
      {
        id = lexical_cast<int>(v.second.get_child("<xmlattr>.id").data());
      }
      catch(const bad_lexical_cast &)
      {
        LOGGING_WARNING( mcLogger, "WARNING: Could not parse sector id!" << endl );
      }
      catch(std::exception& e) {
        LOGGING_WARNING( mcLogger, "WARNING: Could not parse sector id!" << e.what() << endl );
      }
      shared_ptr<AADC_Sector> newSector(new AADC_Sector(id));

      BOOST_FOREACH(const ptree::value_type & vv, v.second){
        if(vv.first == "AADC-Maneuver") {

          int id = 0;


          try
          {
            id = lexical_cast<int>(vv.second.get_child("<xmlattr>.id").data());
          }
          catch(const bad_lexical_cast &)
          {
            LOGGING_WARNING( mcLogger, "WARNING: Could not parse maneuver id!" << endl );
          }
          catch(std::exception& e) {
            LOGGING_WARNING( mcLogger, "WARNING: Could not parse maneuver id!" << endl << e.what() << endl );
          }

          string action = "";
          action = vv.second.get_child("<xmlattr>.action").data();

          std::transform(action.begin(), action.end(), action.begin(), ::tolower); //transform to lower case for better comparison
          //TODO: vertrauen wir dem File oder auch hier Fehlerbehebung?


          //FUN mode for the Kür
          if(action == "fun") {

              LOGGING_INFO( mcLogger, "Found 'fun' command in maneuver list. Starting live mode!" << endl );
              // Create a live maneuver list which listenes to a broker for live commands:
              boost::shared_ptr<ManeuverListLive> temp(new ManeuverListLive());
              temp->mc = mc;

              // Return the maneuver list:
              return static_pointer_cast<IManeuverList>(temp);
          }

          shared_ptr<AADC_Maneuver> newManeuver(new AADC_Maneuver(id, maneuverMap[action], newSector));
          newSector->maneuvers.push_back(newManeuver);
          result->daManeuver.push_back(newManeuver);

        }

      }

      result->sectors.push_back(newSector);
    }



  }


  //Always add a last "Finished Sector"
  shared_ptr<AADC_Sector> finSec(new AADC_Sector(INT32_MAX));
  shared_ptr<AADC_Maneuver> finMan(new AADC_Maneuver(INT32_MAX, MANEUVER_FINISHED,finSec));

  result->daManeuver.push_back(finMan);
  finSec->maneuvers.push_back(finMan);
  result->sectors.push_back(finSec);

  result->dummy = false;
  LOGGING_INFO(mcLogger, "ManList parsed: daManSize: " << result->daManeuver.size() << "sectorSize: " << result->sectors.size()<<endl);

  return static_pointer_cast<IManeuverList>(result);
}

bool ManeuverList::isDummy() {
  return dummy;
}


enumManeuver ManeuverList::getAbsoluteManeuver(int shift) {
  if(shift == 0) { //shortcut
    return daManeuver[0]->action;
  }

  int temp = shift;
  const int size = daManeuver.size() -1;
  temp = max(0,temp);
  temp = min(temp, size);

  return daManeuver[temp]->action;
}

enumManeuver ManeuverList::getCurrentManeuver() {
  return getRelativeManeuver(0);
}

unsigned int ManeuverList::getCurrentSector() {
  return currentSectorPos;
}

enumManeuver ManeuverList::getNextManeuver() {
  return getRelativeManeuver(1);
}

enumManeuver ManeuverList::getPreviosManeuver() {
  return getRelativeManeuver(-1);
}

void ManeuverList::increase() {
  LOGGING_INFO(mcLogger, "ManeuverList::increase()"<<endl);


  if(sectors.size() == 0 || daManeuver.size() == 0) {
    LOGGING_ERROR(mcLogger, "ManeuverList size is somehow zero... should never happen!"<<endl);
    return;
  }

  if(currentSectorPos >= sectors.size() || currentDaManeuver >= daManeuver.size()) {
    LOGGING_ERROR(mcLogger, "ManeuverList size is corrupt. Why?"<<endl);
    return;
  }

  boost::shared_ptr<AADC_Sector> currentSector = sectors[currentSectorPos];

  unsigned int manSize = currentSector->maneuvers.size();

  if(currentManeuverPos +1 < manSize) {
    currentManeuverPos++;
  } else {
    if((currentSectorPos +1) < sectors.size()) { //increase sector
      currentSectorPos++;
      currentManeuverPos = 0;
    }
  }

  if(currentDaManeuver +1 < daManeuver.size()) {
    currentDaManeuver++;
  }


  if(!mc) {
    LOGGING_INFO(mcLogger, "ManList: no access to mission control and cannot communicate with jury" <<endl);
    return;
  }
  if(currentDaManeuver == daManeuver.size() -1) { //parcour finished
    //mc->eventParcourFinished();
    //TODO check of setJuryState in Statemachine is needed!
    mc->setJuryState(stateCar_COMPLETE, daManeuver[currentDaManeuver-1]->id); //one less because the last one is finished
    LOGGING_INFO(mcLogger, "ManList: Parcour finished. Sending complete with manID: " << daManeuver[currentDaManeuver]->id <<endl);
  } else {
    LOGGING_INFO(mcLogger, "ManList: Send Running with manID: " << daManeuver[currentDaManeuver]->id <<endl);
    mc->setJuryState(stateCar_RUNNING, daManeuver[currentDaManeuver]->id);
  }

  //	if(daManeuver[currentDaManeuver]->action == MANEUVER_PARKING_PARALLEL || daManeuver[currentDaManeuver]->action == MANEUVER_PARKING_CROSS) { //parcour finished
  //		mc->eventSearchParking();
  //		LOGGING_INFO(mcLogger, "ManList: search for a parking lot! "<<endl);
  //	}

  //LOGGING_INFO(mcLogger, " Current Maneuver: " << currentDaManeuver <<endl);
}

enumManeuver ManeuverList::getRelativeManeuver(int shift) {

  if(shift == 0) { //shortcut
    return daManeuver[currentDaManeuver]->action;

  }

  int temp = currentDaManeuver + shift;
  const int size = daManeuver.size() -1;
  temp = max(0,temp);
  temp = min(temp, size);

  return daManeuver[temp]->action;


}

string ManeuverList::getDescription() {
  return desc;
}

bool ManeuverList::setManeuverId(int manID) {

  LOGGING_INFO(mcLogger, " [ManList] try to set current maneuver to id: "<< manID <<endl);

  unsigned int max = daManeuver.size();
  unsigned int i = 0;
  unsigned int iRM = 0;
  unsigned int iRS = 0;
  while(i < max) {
    if(daManeuver[i]->id == manID) {
      LOGGING_INFO(mcLogger, " [ManList] setting current maneuver to currentDaManeuver: "<< i << " currentManeuverPos: " << iRM << " currentSectorPos: " << iRS <<endl);
      currentDaManeuver = i;
      currentManeuverPos = iRM;
      currentSectorPos = iRS;
      return true;
    }


    i++;
    if(iRM < sectors[iRS]->maneuvers.size()) {
      iRM++;
    } else {
      iRS++;
      iRM = 0;
    }

  }

  if(manID == 0) {
    LOGGING_INFO(mcLogger, "[ManList] could not find manID 0, but we set current maneuver to the first in the list." <<endl);
    currentDaManeuver = 0;
    currentManeuverPos = 0;
    currentSectorPos = 0;
  }

  LOGGING_INFO(mcLogger, "[ManList] unable to set to manID: " <<manID <<endl);

  return false;
}

std::string ManeuverList::toString() {

  std::stringstream sstr;
  sstr << std::endl << "ManeuverList: " << std::endl;
  // Iterate over all sectors:
  for( size_t secID = 0; secID < sectors.size(); secID ++ )
  {
    boost::shared_ptr<AADC_Sector> sec = sectors[secID];
    sstr << "Sector: " << sec->id << std::endl;
    // Iterate over all maneuvers in the sector:
    for( size_t manID = 0; manID < sec->maneuvers.size(); manID ++ )
    {
      boost::shared_ptr<AADC_Maneuver> man = sec->maneuvers[manID];
      // get maneuver name as string:
      sstr << "\t" << man->id << ": " << man->action << " "
           << maneuverAsString( man->action );
      // Mark the current action:
      if( currentSectorPos == secID && currentManeuverPos == manID )
        sstr << "\t<---";
      sstr << std::endl;
    }
  }
  return sstr.str();
}

std::string ManeuverList::maneuverAsString( enumManeuver man )
{
  switch( man )
  {
    case MANEUVER_LEFT:
      return "left";
    case MANEUVER_RIGHT:
      return "right";
    case MANEUVER_STRAIGHT:
      return "straight";
    case MANEUVER_PARKING_PARALLEL:
      return "parking_parallel";
    case MANEUVER_PARKING_CROSS:
      return "parking_cross";
    case MANEUVER_PULLOUT_LEFT:
      return "pullout_left";
    case MANEUVER_PULLOUT_RIGHT:
      return "pullout_right";
    case MANEUVER_FINISHED:
      return "finished";
  }
  return "";
}

bool ManeuverList::isFinished()
{
  if( getCurrentManeuver() == MANEUVER_FINISHED )
    return true;

  // Just in case, do other checks:
  if( currentSectorPos >= sectors.size() )		// past last sector?
    return true;

  if( currentSectorPos == sectors.size() - 1 )		// last sector?
    if( currentManeuverPos == sectors.back()->maneuvers.size() - 1 )	// last maneuver?
      return true;

  return false;
}

}	// namespace
}	// namespace

/*
 * IManeuverList.h
 *
 *  Created on: Feb 27, 2016
 *      Author: vitali
 */

#ifndef PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_IMANEUVERLIST_H_
#include "MissionControlEnums.h"
#include <string>
#define PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_IMANEUVERLIST_H_

class IManeuverList {
public:
  virtual void increase() = 0;
  virtual std::string toString() = 0;
  virtual unsigned int getCurrentAbsManeuverID() = 0;
  virtual enumManeuver getCurrentManeuver() = 0;
  virtual enumManeuver getPreviosManeuver() = 0;
  virtual bool isFinished() = 0;
  virtual bool setManeuverId(int maneuverEntryID) = 0;
  virtual bool isDummy() = 0;
  virtual bool isLive() { return false; }
  virtual ~IManeuverList() {};
};



#endif /* PACKAGES_OADRIVE_SRC_OADRIVE_MISSIONCONTROL_IMANEUVERLIST_H_ */

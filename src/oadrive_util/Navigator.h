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
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018-10-24
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_UTIL_NAVIGATOR_H
#define OADRIVE_UTIL_NAVIGATOR_H

#include <string>
#include <list>
#include <set>  
#include <map>
#include <tuple>
#include <vector>

#include <math.h>
#include <time.h>
#include <tinyxml.h>

#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_util/utilLogging.h>
#include "oadrive_util/Config.h"

namespace oadrive {
namespace util {

class Navigator
{
public:

  enum NodeType {
    // one input, one output
    NORMAL_1x1,
    NORMAL_2x2,
    NORMAL_3x3,
    // special tiles
    INTERSECTION,
    MERGE,
    PARKING_SPACE 
  };

  // struct representing a single node in the map graph
  struct MapNode {
    MapNode (unsigned int i, oadrive::core::ExtendedPose2d p, float sx, float sy,
             NodeType t, std::vector<unsigned int> l, int psid = 0) :
             id(i), pose(p), sizex(sx), sizey(sy), type(t), lanes(l), parkingSpaceID(psid), obstructed(false) {} 

    unsigned int id; // each node has a unique id
    int parkingSpaceID;
    oadrive::core::ExtendedPose2d pose;
    float sizex, sizey;
    NodeType type;
    std::vector<unsigned int> lanes;  
    bool obstructed;
  };

  // struct representing a single lane node
  struct LaneNode {
    LaneNode (unsigned int i, unsigned int pid, oadrive::core::ExtendedPose2d p,
              std::list<unsigned int> c, std::string m) : 
              id(i), parentID(pid), pose(p), children(c), maneuver(m) {} 

    unsigned int id; // each node has a unique id
    unsigned int parentID; // id of parent node in the map graph
    std::string maneuver;
    oadrive::core::ExtendedPose2d pose;
    std::list<unsigned int> children;
  };

  struct Maneuver {
    Maneuver (std::string n, int e) : name(n), extra(e) {}
    std::string name;
    int extra;
  };

  struct ManeuverNode {
    ManeuverNode (Maneuver m, unsigned int al, unsigned int nl, bool s, bool f) :
                  maneuver(m.name, m.extra), associatedLaneID(al), nextLaneID(nl), seen(s), finished(f), driven(false) {}

    Maneuver maneuver;
    unsigned int associatedLaneID;
    unsigned int nextLaneID;

    bool seen;
    bool finished;
    bool driven;
  };

  struct BusStop {
    BusStop (oadrive::core::ExtendedPose2d p, int id, int ns, int ps) :
             pose(p), id(id), nextStopID(ns), prevStopID(ps) {}   

    oadrive::core::ExtendedPose2d pose;
    int id;
    int nextStopID;
    int prevStopID;
  };

  Navigator(std::string mapPath);

  void reset();

  // generates a maneuver list for a start and a goal in world coordinates
  bool generateManeuverList(oadrive::core::ExtendedPose2d& start,
                       oadrive::core::ExtendedPose2d& goal,
                       std::list<Maneuver>& maneuverList,
                       std::list<oadrive::core::ExtendedPose2d>& wayPoints, bool startIsEndAllowed = true);

  // checks whether pose is close to an illegal tile from which no new path should be planned
  bool isCloseToIllegalTile(oadrive::core::ExtendedPose2d& pose);
  
  bool toggleRoadObstruction(oadrive::core::ExtendedPose2d& targetPose,
                             oadrive::core::ExtendedPose2d& currentPose,
                             oadrive::core::ExtendedPose2d& obstructionPose,
                             std::list<std::tuple<oadrive::core::ExtendedPose2d, float, float>>& obstructedTiles);

  bool isObstructed(oadrive::core::ExtendedPose2d& pose);

  oadrive::core::ExtendedPose2d generateRandomTarget();

  void finishManeuver();

  void connectManeuver();

  void driveManeuver();

  void initializeBusStop();
  oadrive::core::ExtendedPose2d getNextBusStopTarget();

  void getRemainingBusStops(std::list<oadrive::core::ExtendedPose2d>& busStops);

  int getNumberOfBusStops();

  void saveCurrentBusStop();

  void loadBusStop();

private:

  // returns most suitable lane ID for a given pose
  unsigned int getNodeFromPose(oadrive::core::ExtendedPose2d& pose);

  ManeuverNode getLastSeenManeuver();

  // finds shortest path on lane node layer using A*
  bool findShortestPath(unsigned int startLaneID,
                        unsigned int goalLaneID,
                        std::list<unsigned int>& lanes);

  // helper function for A* that reconstructs the shortest path after algorithm is finished
  void reconstructShortestPath(std::map<unsigned int, unsigned int>& cameFromMap,
                               unsigned int currentID,
                               std::list<unsigned int>& shortestPath);

  // converts a node sequence to a maneuver list
  void createManeuverListFromNodes(std::list<unsigned int>& nodes,
                                   std::list<Maneuver>& maneuvers,
                                   std::list<oadrive::core::ExtendedPose2d>& wayPoints);

  std::map<unsigned int, MapNode> idToNode; 
  std::map<unsigned int, LaneNode> idToLaneNode; 
  std::map<unsigned int, std::string> idToName;
  std::map<int, BusStop> idToBusStop;

  std::list<ManeuverNode> maneuverGraph;

  int mCurrentBusStopID = -1;
  int mSavedBusStopID = -1;
};
}
}
#endif // NAVIGATOR_H

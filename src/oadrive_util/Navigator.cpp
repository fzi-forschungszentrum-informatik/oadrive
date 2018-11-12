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

#include "Navigator.h"

using oadrive::util::Config;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive {
namespace util {

Navigator::Navigator(std::string mapPath) {

    // initialize random seed
    srand(time(NULL));

    if (mapPath.substr(0, 1) != "/") {
        mapPath = Config::getConfigPath() + "/" + mapPath;
    }   

    TiXmlDocument doc(mapPath);

	if (!doc.LoadFile())
	{
		LOGGING_ERROR(utilLogger, "UNABLE TO LOAD MAP FILE!");
        return;
	}

    std::map<std::string, unsigned int> nameToId; 

    auto tiles = doc.FirstChildElement("configuration")->FirstChildElement("tiles");

    int idCount = 0;

    // first run: creating ids and general information
    for (auto n = tiles->FirstChild(); n != 0; n = n->NextSibling()) {
        auto tile = n->ToElement();

        // check whether id already exists in map (node is already connected to other node)
        // if yes take exLaneNode(idCount++, node.id, oadrive::core::ExtendedPose2d(0,0,0), children)LaneNode(idCount++, node.id, oadrive::core::ExtendedPose2d(0,0,0), children)isting id as id
        std::string name;

        unsigned int id;
        tile->QueryStringAttribute("id", &name);
       
        // ignore parking space tiles
        if (name.find("Parkflaeche") != std::string::npos) {
            continue;
        }

        if (nameToId.find(name) != nameToId.end()) {
            id = nameToId.at(name);
        }
        else {
            id = idCount++;
            std::cout << "emplacing " << name << std::endl;
            nameToId.emplace(std::make_pair(name, id));
            idToName.emplace(std::make_pair(id, name));
        }

        float x = 0, y = 0, yaw = 0, sizex = 0, sizey = 0;
        std::string isIntersection;
        std::string isMerge;
        tile->QueryFloatAttribute("x", &x);
        tile->QueryFloatAttribute("y", &y);
        tile->QueryFloatAttribute("yaw", &yaw);
        tile->QueryFloatAttribute("sizex", &sizex);
        tile->QueryFloatAttribute("sizey", &sizey);
        tile->QueryStringAttribute("intersection", &isIntersection);
        tile->QueryStringAttribute("mergelane", &isMerge);

        oadrive::core::ExtendedPose2d pose(x, y, yaw);
        NodeType type;
        if (isIntersection == "true") {
            type = NodeType::INTERSECTION;
        }
        else if (isMerge == "true") {
            type = NodeType::MERGE;
        }
        else if (sizex == 2.f && sizey == 2.f) {
            type = NodeType::NORMAL_2x2;
        }
        else if (sizex == 3.f && sizey == 3.f) {
            type = NodeType::NORMAL_3x3;
        }
        else if (sizex == sizey) {
            type = NodeType::NORMAL_1x1;
        }
        // TODO there should be a better way to recognize parking space
        else {
            type = NodeType::PARKING_SPACE;
        }

        std::vector<unsigned int> lanes;

        idToNode.emplace(std::make_pair(id, MapNode(id, pose, sizex, sizey, type, lanes)));
    }

    idCount = 0;
    // second run: creating lanesvector
    for (auto n = tiles->FirstChild(); n != 0; n = n->NextSibling()) {
        auto tile = n->ToElement();

        // get id and corresponding node
        std::string name;
        tile->QueryStringAttribute("id", &name);

        // ignore parking space tiles
        if (name.find("Parkflaeche") != std::string::npos) {
            continue;
        }

        MapNode& node = idToNode.at(nameToId.at(name));
        
        // iterate over lanes
        for (auto l = tile->FirstChild("lane"); l != 0; l = l->NextSibling("lane")) {
            auto lane = l->ToElement();

            unsigned int id = idCount++;
            
            // compute pose (start of lane directed to end of lane)
            float fromX = 0, fromY = 0, toX = 0, toY = 0;
            std::string maneuver = "";
            lane->QueryFloatAttribute("from_x", &fromX);
            lane->QueryFloatAttribute("from_y", &fromY);
            lane->QueryFloatAttribute("to_x", &toX);
            lane->QueryFloatAttribute("to_y", &toY);
            lane->QueryStringAttribute("maneuver", &maneuver);

            float xDir = toX - fromX;
            float yDir = toY - fromY;

            float yaw = atan2(yDir, xDir);

            std::list<unsigned int> children;
            idToLaneNode.emplace(std::make_pair(id, LaneNode(id, node.id, 
                                                             oadrive::core::ExtendedPose2d(fromX,fromY,yaw), children, maneuver)));

            // add lane to map node
            node.lanes.push_back(id);
        }   
    }

    // third run: connect lanes
    for (auto n = tiles->FirstChild(); n != 0; n = n->NextSibling()) {
        auto tile = n->ToElement();

        // get id and corresponding node
        std::string name;
        tile->QueryStringAttribute("id", &name);

        // ignore parking space tiles
        if (name.find("Parkflaeche") != std::string::npos) {
            continue;
        }

        MapNode& node = idToNode.at(nameToId.at(name));

        // iterate over connections
        for (auto c = tile->FirstChild("connection"); c != 0; c = c->NextSibling("connection")) {
            auto connection = c->ToElement();

            // get xml attributes
            int ourLaneIdx = -1;
            int otherLaneIdx = -1;
            std::string otherTileName;
            std::string direction;

            connection->QueryIntAttribute("our_lane", &ourLaneIdx);
            connection->QueryIntAttribute("other_lane", &otherLaneIdx);
            connection->QueryStringAttribute("id", &otherTileName);
            connection->QueryStringAttribute("type", &direction);

            bool outgoing = (direction == "outgoing");

            // ignore parking space tiles
            if (otherTileName.find("Parkflaeche") != std::string::npos) {
                continue;
            }

            MapNode& otherNode = idToNode.at(nameToId.at(otherTileName));
            LaneNode& thisLane = idToLaneNode.at(node.lanes[ourLaneIdx]);
            LaneNode& otherLane = idToLaneNode.at(otherNode.lanes[otherLaneIdx]);

            // only consider outgoing connections for graph building, this should suffice to cover all lane connections
            if (outgoing) {        
                thisLane.children.push_back(otherLane.id);
            }
        }   
    }

    // parking space run
    for (auto p = doc.FirstChild("configuration")->FirstChild("parkingSpace");
         p != 0; p = p->NextSibling("parkingSpace")) {

        auto parkingSpace = p->ToElement();

        int direction=-1 , id=-1;

        float x=0, y= 0;

        parkingSpace->QueryIntAttribute("direction", &direction);
        parkingSpace->QueryIntAttribute("id", &id);
        parkingSpace->QueryFloatAttribute("x", &x);
        parkingSpace->QueryFloatAttribute("y", &y);

        direction = (direction >= 180) ? (direction-180) : (direction+180);
        float yaw = (((float) direction) * M_PI)/180.f;

        int mapNodeID = idToNode.size() + id;
        int laneID = idToLaneNode.size() + id;

        // create the map node
        oadrive::core::ExtendedPose2d mapNodePose(x,y,0.f);
        std::vector<unsigned int> lanes;
        lanes.push_back(laneID);
        std::stringstream name;
        name << "ParkingSpace" << id;

        idToNode.emplace(std::make_pair(mapNodeID, MapNode(mapNodeID, mapNodePose, 0.5, 0.5, NodeType::PARKING_SPACE, lanes, id)));
        idToName.emplace(std::make_pair(mapNodeID, name.str()));

        // find lanes that should be connected to parking space
        std::list<unsigned int> children;
        
        // find closest tile
        unsigned int bestTileID = -1;
        float minDist = std::numeric_limits<float>::max();
        for (std::pair<unsigned int, MapNode> p : idToNode) {
            MapNode& node = p.second;

            if (node.type == NodeType::PARKING_SPACE) {
                continue;
            }

            float distance = sqrt(pow(x-node.pose.getX(),2.f) + pow(y-node.pose.getY(),2.f));

            if (distance <= minDist) {
                bestTileID = node.id;
                minDist = distance;
            }
        }

        // add all lanes of tile to children and also find closest lane to add parking space as its child
        unsigned int bestLaneID = -1;
        minDist = std::numeric_limits<float>::max();

        MapNode& closestNode = idToNode.at(bestTileID);
        for (auto l : closestNode.lanes) {
            LaneNode& lane = idToLaneNode.at(l);

            children.push_back(l);

            float distance = sqrt(pow(x-lane.pose.getX(),2.f) + pow(y-lane.pose.getY(),2.f));

            if (distance <= minDist) {
                bestLaneID = lane.id;
                minDist = distance;
            }
        }

        // add parking space lane to children of closest lane
        LaneNode& bestLane = idToLaneNode.at(bestLaneID);
        bestLane.children.push_back(laneID);

        // create the lane node 
        oadrive::core::ExtendedPose2d lanePose(x,y,yaw);
        idToLaneNode.emplace(std::make_pair(laneID, LaneNode(laneID, mapNodeID, lanePose, children, "")));
    }   

    // first bus stop run, create objects
    int internalID = 0;
    for (auto p = doc.FirstChild("configuration")->FirstChild("busStop");
         p != 0; p = p->NextSibling("busStop")) {

        auto busStop = p->ToElement();
    
        int direction=-1 , id=-1;

        float x=0, y= 0;

        busStop->QueryIntAttribute("direction", &direction);
        busStop->QueryIntAttribute("id", &id);
        busStop->QueryFloatAttribute("x", &x);
        busStop->QueryFloatAttribute("y", &y);

        direction = (direction >= 180) ? (direction-180) : (direction+180);
        float yaw = (((float) direction) * M_PI)/180.f;

        oadrive::core::ExtendedPose2d busStopPose(x,y,yaw);
        idToBusStop.emplace(std::make_pair(id, BusStop(busStopPose, id, -1, -1)));

        internalID++;
    }

    // second bus stop run, connect objects 
    int highestBusStopID = internalID - 1;
    for (auto& p : idToBusStop) {
        p.second.prevStopID = (p.first == 0) ? highestBusStopID : p.first - 1;
        p.second.nextStopID = (p.first == highestBusStopID) ? 0 : p.first + 1;
    }

    // debug output
    for (std::pair<unsigned int, LaneNode> p: idToLaneNode) {
        std::cout << "id: " << p.first << " =>" << p.second.pose << " , successors:";
        
        for (auto id : p.second.children    ) {
            std::cout << id << ", ";
        }
        std::cout << '\n';
    }
}

void Navigator::reset() {
    maneuverGraph.clear();
}

unsigned int Navigator::getNodeFromPose(oadrive::core::ExtendedPose2d& pose) {

    float minDist = std::numeric_limits<float>::max();
    unsigned int bestNodeId = -1;

    for (std::pair<unsigned int, MapNode> p: idToNode) {
        MapNode node = p.second;

        float x = node.pose.getX();
        float y = node.pose.getY();

        float lowerX = x - (node.sizex/2.f);
        float upperX = x + (node.sizex/2.f);
        float lowerY = y - (node.sizey/2.f);
        float upperY = y + (node.sizey/2.f);
  
        // check if pose is in bounds of current map tile
        if (lowerX <= pose.getX() && pose.getX() <= upperX &&
            lowerY <= pose.getY() && pose.getY() <= upperY) {
            
            // if yes return the corresponding node id
            bestNodeId = node.id;
            break;
        }

        // keep track of closest map tile in case pose is not in bounds of any tile
        float distance = sqrt(pow(x-pose.getX(),2.f) + pow(y-pose.getY(),2.f));

        if (distance <= minDist) {
            minDist = distance;
            bestNodeId = node.id;
        }
    }

    const MapNode& bestNode = idToNode.at(bestNodeId);

    minDist = std::numeric_limits<float>::max();
    unsigned int bestLaneId = -1;

    // iterate over lanes in node
    for (unsigned int laneID : bestNode.lanes) {
        const LaneNode& lane = idToLaneNode.at(laneID);

        const float laneYaw = lane.pose.getYaw();

        // find shortest absolute angle distance between car yaw and lane yaw
        float angleDist = fabs(atan2(sin(laneYaw-pose.getYaw()), cos(laneYaw-pose.getYaw())));        

        if (angleDist <= minDist) {
            minDist = angleDist;
            bestLaneId = lane.id;
        }
    }

    return bestLaneId;
}

bool Navigator::findShortestPath(unsigned int startLaneID,
                                 unsigned int goalLaneID,
                                 std::list<unsigned int>& lanes) {

    lanes.clear();

    // A*, implementation taken from wikipedia
    std::set<unsigned int> closedSet;
    std::set<unsigned int> openSet;
    openSet.insert(startLaneID);

    std::map<unsigned int, unsigned int> cameFromMap;
    std::map<unsigned int, int> gScore;
    std::map<unsigned int, int> fScore;

    // intitialize the maps with 'infinity'
    for (std::pair<unsigned int, LaneNode> p: idToLaneNode) {
        unsigned int currentID = p.first;

        gScore[currentID] = std::numeric_limits<int>::max();
        fScore[currentID] = std::numeric_limits<int>::max();
    }
    gScore[startLaneID] = 0;
    fScore[startLaneID] = 0; // this should be the heuristic value, if heuristic is used

    // main loop
    while (!openSet.empty()) {

        // find node in set with lowest f score
        unsigned int currentID = -1;
        int lowestFScore = std::numeric_limits<int>::max();
        for (unsigned int id : openSet) {
            if (fScore[id] < lowestFScore) {
                lowestFScore = fScore[id];
                currentID = id;
            }
        }

        // if goal is reached algorithm is finished an path needs to be reconstructed
        if (currentID == goalLaneID) {
            std::cout << "RECONSTRUCTING" << std::endl;
            reconstructShortestPath(cameFromMap, currentID, lanes);
            return (!lanes.empty());
        } 

        // remove current node from sets
        openSet.erase(currentID);
        closedSet.insert(currentID);

        if ((idToNode.at((idToLaneNode.at(currentID)).parentID)).obstructed) {
            continue;
        }

        // iterate over all neighbors
        for (unsigned int neighborID : idToLaneNode.at(currentID).children) {
            
            // if neighbor is already closed, skip it
            if (closedSet.find(neighborID) != closedSet.end()) {
                continue;
            } 

            // get 'length' of current node
            // TODO this should be moved to a another place
            int distToNeighbor = 999;
            MapNode& currentNode = idToNode.at(idToLaneNode.at(currentID).parentID);
            switch (currentNode.type) {
                case NodeType::NORMAL_1x1:
                    distToNeighbor = 1;
                    break;
                case NodeType::NORMAL_2x2:
                    distToNeighbor = 2;
                    break;
                case NodeType::NORMAL_3x3:
                    distToNeighbor = 3;
                    break;
                case NodeType::INTERSECTION:
                    distToNeighbor = 1;
                    break;
                case NodeType::MERGE:
                    distToNeighbor = 4;
                    break;
                case NodeType::PARKING_SPACE:
                    distToNeighbor = 3; 
                    break;
                default:
                    break;
            }

            int tentativeGScore = gScore[currentID] + distToNeighbor;

            // if neighbor is not already in open set, add it
            if (openSet.find(neighborID) == openSet.end()) {
                openSet.insert(neighborID);
            }
            // if the new path to the node is worse, then skipt it
            else if (tentativeGScore >= gScore[neighborID]) {
                continue;
            }

            // record the best path to this node
            cameFromMap[neighborID] = currentID;
            gScore[neighborID] = tentativeGScore;
            // the 0 here is the heuristic value. because of the simplicity of the problem
            // we do not use a heuristic, so essentially this is a dijkstra algorithm and the 
            // f score is equal to the g score
            fScore[neighborID] = gScore[neighborID] + 0;
        }
    }

    return false;
}

void Navigator::reconstructShortestPath(std::map<unsigned int, unsigned int>& cameFromMap,
                                        unsigned int currentID,
                                        std::list<unsigned int>& shortestPath) {
    
    shortestPath.push_front(currentID);
    
    while (cameFromMap.find(currentID) != cameFromMap.end()) {
        currentID = cameFromMap[currentID];
        shortestPath.push_front(currentID);
    }
}

void Navigator::createManeuverListFromNodes(std::list<unsigned int>& nodes,
                                            std::list<Maneuver>& maneuvers,
                                            std::list<oadrive::core::ExtendedPose2d>& wayPoints) {
    

    oadrive::core::Pose2d currentPose;

    unsigned int prevID = -1;

    int i = 0;

    for(auto id : nodes) {
        
        if (prevID == -1) {
            prevID = id;
            i++;
            continue;
        }

        const LaneNode& prevLane = idToLaneNode.at(prevID);
        const MapNode& prevNode = idToNode.at(prevLane.parentID);

        const LaneNode& lane = idToLaneNode.at(id);
        const MapNode& node = idToNode.at(lane.parentID);

        // if (id == 1 && prevNode.type != NodeType::PARKING_SPACE) {
        //     prevID = id;
        //     i++;
        //     continue;
        // }

        wayPoints.push_back(lane.pose);

        float prevYaw = prevLane.pose.getYaw();
        float newYaw = lane.pose.getYaw();
        float angleDist = atan2(sin(prevYaw-newYaw), cos(prevYaw-newYaw));
        
        bool newManeuver = false;

        switch (prevNode.type) {
            case NodeType::NORMAL_1x1:
            case NodeType::NORMAL_2x2:
            case NodeType::NORMAL_3x3:
                break;
            case NodeType::INTERSECTION:
            case NodeType::MERGE:
                if(prevLane.maneuver != "" ) {
                    maneuvers.push_back(Maneuver(prevLane.maneuver, -1));
                    newManeuver = true;
                }
                break;
            case NodeType::PARKING_SPACE:
                maneuvers.push_back(Maneuver("cross_parking", prevNode.parkingSpaceID)); 
                if (angleDist > 0.f) {
                    maneuvers.push_back(Maneuver("pull_out_left", -1));
                }
                else if (angleDist < 0.f) {
                    maneuvers.push_back(Maneuver("pull_out_right", -1));
                }
                break;
            default:
                break;
        }

        if (newManeuver) {
            Maneuver m = maneuvers.back();
            bool seen = (m.name == "merge_left");
            ManeuverNode node(m, prevID, id, seen, false);
            maneuverGraph.push_back(node);
        }

        newManeuver = false;
        if (i == (nodes.size()-1)) {
            switch (node.type) {
                case NodeType::NORMAL_1x1:
                case NodeType::NORMAL_2x2:
                case NodeType::NORMAL_3x3:
                    break;
                case NodeType::INTERSECTION:
                case NodeType::MERGE:
                    if(lane.maneuver != "") {
                        maneuvers.push_back(Maneuver(lane.maneuver, -1));
                        newManeuver = true;
                    }
                    break;
                case NodeType::PARKING_SPACE:
                    maneuvers.push_back(Maneuver("cross_parking", node.parkingSpaceID)); 
                    break;
                default:
                    break;
            }
        }

        if (newManeuver) {
            Maneuver m = maneuvers.back();
            bool seen = (m.name == "merge_left");
            ManeuverNode node(m, id, id, seen, false);
            maneuverGraph.push_back(node);
        }

        prevID = id;
        i++;
    }

}

bool Navigator::isCloseToIllegalTile(oadrive::core::ExtendedPose2d& pose) {

    for (std::pair<unsigned int, MapNode> p: idToNode) {
        
        MapNode node = p.second;

        if (node.type == NodeType::NORMAL_1x1 || node.type == NodeType::NORMAL_2x2 || node.type == NodeType::NORMAL_3x3) {
            continue;
        }

        float x = node.pose.getX();
        float y = node.pose.getY();

        // keep track of closest map tile in case pose is not in bounds of any tile
        float distance = sqrt(pow(x-pose.getX(),2.f) + pow(y-pose.getY(),2.f));

        if (node.type == NodeType::MERGE) {

            if (distance <= 2.f) {
                return true;
            }
        }
        else if (node.type == NodeType::INTERSECTION) {
            if (distance <= 1.5f) {
                return true;
            }
        }
    }
    
    return false;
}

bool Navigator::isObstructed(oadrive::core::ExtendedPose2d& obstructionPose) {
    unsigned int obstructionTargetID = idToLaneNode.at(getNodeFromPose(obstructionPose)).parentID;
    MapNode& node = idToNode.at(obstructionTargetID);
    return node.obstructed;
}

bool Navigator::toggleRoadObstruction(oadrive::core::ExtendedPose2d& targetPose,
                             oadrive::core::ExtendedPose2d& currentPose,
                             oadrive::core::ExtendedPose2d& obstructionPose,
                             std::list<std::tuple<oadrive::core::ExtendedPose2d, float, float>>& obstructedTiles) {

    unsigned int obstructionTargetID = idToLaneNode.at(getNodeFromPose(obstructionPose)).parentID;
    unsigned int startID = idToLaneNode.at(getNodeFromPose(currentPose)).parentID;
    unsigned int goalID = idToLaneNode.at(getNodeFromPose(targetPose)).parentID;
    
    // if (obstructionTargetID == startID) {
    //     std::cout << "ERROR: Cannot obstruct tile car is currently standing on!" << std::endl;
    //     return false;
    // }
    // if (obstructionTargetID == goalID) {
    //     std::cout << "ERROR: Cannot obstruct goal tile!" << std::endl;
    //     return false;
    // }

    MapNode& node = idToNode.at(obstructionTargetID);

    if (node.type == MERGE) {
        std::cout << "WARN: Merge lane cannot be obstructed!" << std::endl;
        return true;
    }

    node.obstructed = !node.obstructed;

    obstructedTiles.clear();
    for (std::pair<unsigned int, MapNode> p : idToNode) {
        
        MapNode curNode = p.second;

        if(curNode.obstructed) {
            obstructedTiles.push_back(std::make_tuple(curNode.pose, curNode.sizex, curNode.sizey));
        }
    }

    if (startID == obstructionTargetID) {
        return false;
    }
    
    // if the new obstruction happens on a successor lane of the current lane, return false for activating teleop
    for (auto childID : idToLaneNode.at(getNodeFromPose(currentPose)).children) {
        
        if (idToLaneNode.at(childID).parentID == obstructionTargetID) {
            return false;
        }
    }

    return true;
}

oadrive::core::ExtendedPose2d Navigator::generateRandomTarget() {
    bool success = false;            
            
    oadrive::core::ExtendedPose2d pose(0,0,0);
    do {
        auto it = idToLaneNode.begin();
        std::advance(it, rand() % idToLaneNode.size());
        LaneNode& lane = it->second;
        MapNode& node = idToNode.at(lane.parentID);

        if (node.type == NodeType::INTERSECTION || node.type == NodeType::MERGE) {
            continue;
        }

        pose = oadrive::core::ExtendedPose2d(node.pose.getX(), node.pose.getY(), lane.pose.getYaw());
        success = true;

    } while (!success);

    return pose;
}

void Navigator::driveManeuver() {
    for (ManeuverNode& m : maneuverGraph) {
        if (m.driven) {
            continue;
        }

        if (!m.finished) {
            std::cout << "ERROR: Tried to mark maneuver as driven that is not finished yet" << std::endl;
            break;
        }

        m.driven = true;
        break;
    }
}

void Navigator::finishManeuver() {
    for (ManeuverNode& m : maneuverGraph) {
        if (m.finished) {
            continue;
        }

        m.finished = true;
       
        if (m.maneuver.name == "merge_left") {
            m.driven = true;
        }
        
        break;
    }
}

void Navigator::connectManeuver() {
    for (ManeuverNode& m : maneuverGraph) {
        if (m.seen) {
            continue;
        }

        m.seen = true;

        break;
    }
}

void Navigator::initializeBusStop() {
    mCurrentBusStopID = 0;
}

oadrive::core::ExtendedPose2d Navigator::getNextBusStopTarget() {
    BusStop& busStop = idToBusStop.at(mCurrentBusStopID);
    BusStop& nextStop = idToBusStop.at(busStop.nextStopID);

    mCurrentBusStopID = busStop.nextStopID;

    oadrive::core::ExtendedPose2d pose = nextStop.pose;

    std::cout << "Now driving to next bus stop with id " << nextStop.id << " a position (" 
              << pose.getX() << "," << pose.getY() << ")" << std::endl;

    return pose;
}

void Navigator::getRemainingBusStops(std::list<oadrive::core::ExtendedPose2d>& busStops) {
    for (auto& p : idToBusStop) {
        if (p.first == mCurrentBusStopID) {
            continue;
        }

        busStops.push_back(p.second.pose);
    }
}


int Navigator::getNumberOfBusStops() {
    return idToBusStop.size();
}


void Navigator::saveCurrentBusStop() {
    mSavedBusStopID = mCurrentBusStopID;
}


void Navigator::loadBusStop() {
    mCurrentBusStopID = mSavedBusStopID;
}

Navigator::ManeuverNode Navigator::getLastSeenManeuver() {
    
    ManeuverNode last = ManeuverNode(Maneuver("",-1), -1, -1, false, false);

    last.associatedLaneID = -1;
    
    for (ManeuverNode& m : maneuverGraph) {
        if (!m.finished && !m.seen) {
            break;
        }

        last = m;
    }

    return last;
}

bool Navigator::generateManeuverList(oadrive::core::ExtendedPose2d& start,
                                     oadrive::core::ExtendedPose2d& goal,
                                     std::list<Navigator::Maneuver>& maneuverList,
                                     std::list<oadrive::core::ExtendedPose2d>& wayPoints,
                                     bool startIsEndAllowed) {

    maneuverList.clear();    
    wayPoints.clear();

    // reduce maneuver graph to only keep relevant maneuvers
    // TODO!
    while (maneuverGraph.size() > 0 && !maneuverGraph.back().seen) {
        maneuverGraph.pop_back();
    }
    // Remove old, useless maneuvers
    while (maneuverGraph.size() > 2) {
        maneuverGraph.pop_front();
    }

    std::cout << "Keeping following maneuvers" << std::endl;
    for (ManeuverNode& n : maneuverGraph) {
        std::cout << n.maneuver.name << ", " << (n.seen ? "seen"  : "unseen") << std::endl;
    }
                             
    //oadrive::core::ExtendedPose2d startPose(-1.875f, -3.125f, 3.1f);
    //oadrive::core::ExtendedPose2d goalPose(1.f, -6.f, 3.1f);

    std::cout << "Computing closest tiles..." << std::endl;

    ManeuverNode startManeuverNode = getLastSeenManeuver();

    auto nodeFromPoseID = getNodeFromPose(start);

    // if there is a seen maneuver, start planning only right after that maneuver is finished
    unsigned int startID = -1;
    if ((startManeuverNode.associatedLaneID == -1 || startManeuverNode.driven) && startManeuverNode.associatedLaneID != nodeFromPoseID) {
        std::cout << "Choosing car pose as start" << std::endl;
        startID = nodeFromPoseID;
    }
    else {
        std::cout << "Choosing first unseen lane as start" << std::endl;
        startID = startManeuverNode.nextLaneID;
    }

    // if maneuvernode has no valid next lane id, the use car position
    if (startID == -1) {
        std::cout << "ERROR: First unseen tile has no valid next lane connected to it. choosing car pose instead" << std::endl;
        startID = getNodeFromPose(start);
    }

    unsigned int goalID = getNodeFromPose(goal);

    LaneNode& startLane = idToLaneNode.at(startID);
    LaneNode& goalLane = idToLaneNode.at(goalID);
    MapNode& startNode = idToNode.at(startLane.parentID);
    MapNode& goalNode = idToNode.at(goalLane.parentID);

    if (idToNode.at(idToLaneNode.at(getNodeFromPose(start)).parentID).obstructed) {
        return false;
    }
    if (goalNode.obstructed) {
        std::cout << "ERROR: TARGET TILE IS OBSTRUCTED." << std::endl;
        return false;
    }
    if (startNode.obstructed) {
        std::cout << "ERROR: SOURCE TILE IS OBSTRUCTED." << std::endl;
        return false;
    }

    if (getNodeFromPose(start) == goalID) {
        std::cout << "Cant choose start tile as target, so choosing adjacent tile" << std::endl;
        
        /*startLane = idToLaneNode.at(startLane.children.front());
        startNode = idToNode.at(startLane.parentID);

        if (startNode.obstructed) {
            return false;
        }*/

        return startIsEndAllowed;
    }

    std::cout << "Computing shortest path..." << std::endl;

    std::list<unsigned int> shortestPath;
    bool success = findShortestPath(startID, goalID, shortestPath);

    if(!success) {
        std::cout << "NO PATH FOUND!" << std::endl;
        return false;
    }

    std::cout << std::endl << "SHORTEST PATH FROM " << idToName[startNode.id] << " TO " << idToName[goalNode.id] << std::endl;
    for (auto id : shortestPath) {
        LaneNode& lane = idToLaneNode.at(id);
        MapNode& node = idToNode.at(lane.parentID);
        std::cout << "Tile: " << idToName[node.id] << ", Type: " << node.type << std::endl;
    }

    std::cout << "Building maneuver list" << std::endl;

    ManeuverNode lastManeuver = getLastSeenManeuver();
    if (lastManeuver.associatedLaneID != -1 && !lastManeuver.finished) {
        maneuverList.push_front(lastManeuver.maneuver);
    }

    createManeuverListFromNodes(shortestPath, maneuverList, wayPoints);

    std::cout << std::endl << "MANEUVER LIST: " << std::endl;
    for (auto& maneuver : maneuverList) {
        std::cout << "Name: " << maneuver.name << ", Extra: " << maneuver.extra << std::endl;
    }

    std::cout << "DONE" << std::endl;

    return true;
}

}
}
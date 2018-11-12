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
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_ROADPERCEPTION_MAP_H
#define OADRIVE_ROADPERCEPTION_MAP_H

#include <string>
#include <vector>
#include <iostream>
#define TIXML_USE_TICPP
#include <tinyxml.h>
#include <string>

#include "../FeatureDetection/StreetTypes.h"

namespace oadrive
{
namespace lanedetection
{


class Map
{

public:

    class Tile {
    public:
        OadrivePose pose;
        bool isIntersection;
        bool isMerge;

        Tile(float x, float y, float yaw, bool isIntersection, bool isMerge)
            : pose(x, y, yaw) {
            this->isIntersection = isIntersection;
            this->isMerge = isMerge;
        }

        std::string toString() {
            std::ostringstream stringStream;
            stringStream << "[TILE x=" << pose.getX() << ", y=" << pose.getY() << ", yaw=" << pose.getYaw() << ", intersection=" << isIntersection << ", merge=" << isMerge << "]";
            return stringStream.str();
        }
    };

    Map(std::string mapPath);

    void loadFromString(std::string data);
    void loadMap(TiXmlDocument &doc);

    std::vector<Tile> queryIntersections(OadrivePose pose, float radius = 2.0, float maxAngle = M_PI_2);
    std::vector<Tile> queryMergeLanes(OadrivePose pose, float radius = 2.0, float maxAngle = M_PI_2, float maxY = -1.0);
    std::vector<Tile> query(OadrivePose pose, float radius = 2.0, float maxAngle = M_PI_2, bool intersections=true, bool merge = false, float maxY = -1.0);

    bool hasRamp();
    OadrivePose getRampTakeOff();

private:
    std::vector<Tile> mTiles;

    void addTile(Tile tile) {
        mTiles.push_back(tile);
    }

};

}    // namespace
}    // namespace




#endif

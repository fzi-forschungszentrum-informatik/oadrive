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

#include "MapPatcher.h"
#include <oadrive_lanedetection/lanedetectionLogging.h>
#include "oadrive_util/Config.h"
#include "oadrive_world/TrajectoryFactory.h"

using oadrive::util::ImageHelper;
using oadrive::util::Config;
using oadrive::world::TrafficSign;
using icl_core::logging::endl;
using icl_core::logging::flush;

namespace oadrive
{
namespace lanedetection
{

Map::Map(std::string mapPath)
{
    // Relative to config folder?
    if (mapPath.substr(0, 1) != "/") {
        mapPath = Config::getConfigPath() + "/" + mapPath;
    }

    TiXmlDocument doc(mapPath);

	if (!doc.LoadFile())
	{
		LOGGING_ERROR(lanedetectionLogger, "UNABLE TO LOAD MAP FILE!");
        return;
	}

    loadMap(doc);

  //exit(0);
}

void Map::loadFromString(std::string data) {
    TiXmlDocument doc;
    doc.Parse(data.c_str(), 0, TIXML_ENCODING_UTF8);

    loadMap(doc);
}

void Map::loadMap(TiXmlDocument &doc) {
    mTiles.clear();

    auto tiles = doc.FirstChildElement("configuration")->FirstChildElement("tiles");

    for (auto node = tiles->FirstChild(); node != 0; node = node->NextSibling()) {
        auto tile = node->ToElement();
        float x = 0, y = 0, yaw = 0;
        std::string isIntersection;
        std::string isMerge;
        tile->QueryFloatAttribute("x", &x);
        tile->QueryFloatAttribute("y", &y);
        tile->QueryFloatAttribute("yaw", &yaw);
        tile->QueryStringAttribute("intersection", &isIntersection);
        tile->QueryStringAttribute("mergelane", &isMerge);

        addTile(Tile(x, y, yaw, isIntersection == "true", isMerge == "true"));
    }

    std::cout << "Loaded map: " << std::endl;
    for (auto tile : mTiles) {
        std::cout << tile.toString() << std::endl;
    }
}

std::vector<Map::Tile> Map::queryIntersections(OadrivePose pose, float radius, float maxAngle) {
    return this->query(pose, radius, maxAngle, true, false);
}

std::vector<Map::Tile> Map::queryMergeLanes(OadrivePose pose, float radius, float maxAngle, float maxY) {
    return this->query(pose, radius, maxAngle, false, true, maxY);
}

std::vector<Map::Tile> Map::query(OadrivePose pose, float radius, float maxAngle, bool intersections, bool merge, float maxY) {
    std::vector<Tile> result;

    for (auto tile : mTiles) {
        if ((intersections && tile.isIntersection) || (merge && tile.isMerge)) {
            if (tile.pose.distance(pose) < radius) {
                // Norm vec pose:
                float normalVectorX = 1.0 * cos(pose.getYaw());
                float normalVectorY = 1.0 * sin(pose.getYaw());

                float posToTileX = tile.pose.getX() - pose.getX();
                float posToTileY = tile.pose.getY() - pose.getY();

                auto poseToTileVec = OadrivePose(posToTileX, posToTileY, 0);
                float posToTileLen = poseToTileVec.length();

                float angle = acos((normalVectorX * posToTileX + normalVectorY * posToTileY) / posToTileLen);

                if (maxY >= 0) {
                    // we have to rotate poseToTileVec into the car coordinate system, then we just take y
                    // To rotate it, we have to rotate it with -pose.yaw

                    auto rotated = poseToTileVec.rotate(-pose.getYaw());
                    float dy = rotated.getY();

                    // std::cout << dy << std::endl; 

                    if (fabs(dy) > maxY) {
                        continue;
                    }
                }

                // std::cout << "Angel is " << angle << std::endl;
                
                if (fabs(angle) < maxAngle) {
                    // std::cout << tile.toString() << std::endl;
                    result.push_back(tile);
                }
            }
        }

    }

    return result;
}

bool Map::hasRamp() {
    for (auto tile : mTiles) {
        if (tile.isMerge) {
            return true;
        }
    }

    return false;
}

OadrivePose Map::getRampTakeOff() {
    const float xOffset = -3 + 0.25 + RAMP_START_OFFSET;
    const float yOffset = 4;

    for (auto tile : mTiles) {
        if (tile.isMerge) {
            const float tYaw = tile.pose.getYaw() - M_PI/2;
            // rotate tile local offsets into world:
            const float rotXOffset = xOffset * cos(tYaw) - yOffset * sin(tYaw);
            const float rotYOffset = xOffset * sin(tYaw) + yOffset * cos(tYaw);
            
            return OadrivePose(tile.pose.getX() + rotXOffset, tile.pose.getY() + rotYOffset, tYaw);
        }
    }

    return OadrivePose();
}

}
}
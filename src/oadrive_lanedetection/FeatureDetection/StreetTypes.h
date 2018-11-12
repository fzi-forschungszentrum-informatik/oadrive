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
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2015-11-01
 *
 */
//----------------------------------------------------------------------

/*! Defines types and enums needed for street patch detection. */

#ifndef OADRIVE_LANEDETECTION_STREETTYPES_H
#define OADRIVE_LANEDETECTION_STREETTYPES_H

#include <map>
#include <list>
#include <oadrive_core/ExtendedPose2d.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace oadrive
{
namespace lanedetection
{

enum FeatureType
{
  CENTER_LINE = 0,
  LEFT_SIDE_LINE = 1,
  RIGHT_SIDE_LINE = 2,
  STREET_LINE = 3,
  CROSSROAD_LINE = 4,
  PARKING_LOT = 5,
  CROSSROAD_CORNER = 6,
  PARKING_LINE = 7,
  UNKNOWN = 8
};
const int numFeatureTypes = 9;

struct OadrivePose
{
private:

  float m_x;
  float m_y;
  float m_yaw;

public:

  float getX() const { return m_x; };

  void setX(float x) { m_x = x; };


  float getY() const { return m_y; };

  void setY(float y) { m_y = y; };

  float getYaw() const { return m_yaw; };

  void setYaw(float yaw) { m_yaw = yaw; };

  OadrivePose(float x, float y, float yaw) : m_x(x), m_y(y), m_yaw(yaw) {};

  OadrivePose() : m_x(0.f), m_y(0.f), m_yaw(0.f) {};

  float getAngleTo(OadrivePose other)
  {
    float normalVectorX = 1.0 * cos(this->getYaw());
    float normalVectorY = 1.0 * sin(this->getYaw());

    float posToTileX = other.getX() - this->getX();
    float posToTileY = other.getY() - this->getY();

    auto poseToTileVec = OadrivePose(posToTileX, posToTileY, 0);
    float posToTileLen = poseToTileVec.length();

    return acos((normalVectorX * posToTileX + normalVectorY * posToTileY) / posToTileLen);
  }

  float orientedDistance(OadrivePose other)
  {
    float dist = sqrt(pow(m_x - other.m_x, 2.f) + pow(m_y - other.m_y, 2.f));

    if (fabs(this->getAngleTo(other)) < M_PI) {
      return dist;
    } else {
      return -dist;
    }
  }

  float distance(OadrivePose other)
  {
    return sqrt(pow(m_x - other.m_x, 2.f) + pow(m_y - other.m_y, 2.f));
  }

  float length() 
  {
    return sqrt(pow(m_x, 2.f) + pow(m_y, 2.f));
  }

  OadrivePose rotate(float yaw) {
    float x = getX() * cos(yaw) - getY() * sin(yaw);
    float y = getX() * sin(yaw) + getY() * cos(yaw);
    return OadrivePose(x, y, getYaw() + yaw);
  }
};


/*! A feature is found in images by the HaarFilter class. It represent a point where a 
 * certain kind of line (Side-line, center-line etc. was found). */
struct Feature
{
  FeatureType type;
  OadrivePose localPose;
  OadrivePose poseRelPatch;
  float probability;

  Feature() {};

  Feature(OadrivePose localPose) { localPose = localPose; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

typedef std::vector<Feature, Eigen::aligned_allocator<Feature> > FeatureVector;
typedef std::vector<Feature*, Eigen::aligned_allocator<Feature*> > FeaturePointerVector;

/*! Features and patches can vote for patches in a "Hough" like space. (Inspired by the generalized
 * Hough Transform). Beforehand, Votes are defined for every kind of patch. For example, a center
 * line will have a vote for a straight patch which is at the position where the center line is.
 * Higher weights for votes will give the voting feature/patch more importance. For example, a
 * Corner feature or T-Feature could be considered "more important" because it contains a lot of
 * information, so the weights of this feature's Vote could be higher. */
struct Vote
{
  cv::Point2f pos;
  cv::Size2f size;
  unsigned int weight;
  float angleOffset;

  Vote(cv::Point2f pos, cv::Size2f size, unsigned int weight, float angleOffset) :
          pos(pos), size(size), weight(weight), angleOffset(angleOffset) {}

};

}
}

#endif

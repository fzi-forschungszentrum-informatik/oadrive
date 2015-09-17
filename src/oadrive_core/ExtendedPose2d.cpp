// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the Open Autonomous Driving Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2015 FZI Forschungszentrum Informatik, Karlsruhe, Germany

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Sebastian Klemm <klemm@fzi.de>
 * \date    2014-03-07
 *
 */
//----------------------------------------------------------------------
#include "ExtendedPose2d.h"

namespace oadrive {
namespace core {

ExtendedPose2d::ExtendedPose2d()
  : m_pose(Pose2d::Identity()),
    m_curvature(std::numeric_limits<double>::quiet_NaN()),
    m_velocity(std::numeric_limits<double>::quiet_NaN())
{
}

ExtendedPose2d::ExtendedPose2d(const Pose2d& pose)
  : m_pose(pose),
    m_curvature(std::numeric_limits<double>::quiet_NaN()),
    m_velocity(std::numeric_limits<double>::quiet_NaN())
{
}

ExtendedPose2d::ExtendedPose2d(double x, double y, double yaw)
  : m_curvature(std::numeric_limits<double>::quiet_NaN()),
    m_velocity(std::numeric_limits<double>::quiet_NaN())
{
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(m_pose, x, y, yaw);
}

ExtendedPose2d::~ExtendedPose2d()
{
}

double ExtendedPose2d::getX() const
{
  return m_pose.translation()[0];
}

double ExtendedPose2d::getY() const
{
  return m_pose.translation()[1];
}

double ExtendedPose2d::getYaw() const
{
  return PoseTraits<Pose2d>::yaw(m_pose);
}

double ExtendedPose2d::getCurvature() const
{
  return m_curvature;
}

double ExtendedPose2d::getVelocity() const
{
  return m_velocity;
}

Position2d ExtendedPose2d::getPosition() const
{
  return m_pose.translation();
}

Eigen::Matrix2d ExtendedPose2d::getOrientation() const
{
  return m_pose.rotation();
}

const Pose2d& ExtendedPose2d::pose() const
{
  return m_pose;
}

Pose2d ExtendedPose2d::getPose() const
{
  return m_pose;
}

void ExtendedPose2d::setX(double x)
{
  m_pose.translation()[0] = x;
}

void ExtendedPose2d::setY(double y)
{
  m_pose.translation()[1] = y;
}

void ExtendedPose2d::setPosition(double x, double y)
{
  m_pose.translation()[0] = x;
  m_pose.translation()[1] = y;
}

void ExtendedPose2d::setPosition(const Position2d& position)
{
  m_pose.translation() = position;
}

void ExtendedPose2d::setYaw(double yaw)
{
  PoseTraits<Pose2d>::fromOrientationRPY(m_pose,
                                         yaw);
}

void ExtendedPose2d::setCurvature(double curvature)
{
  m_curvature = curvature;
}

void ExtendedPose2d::setVelocity(double velocity)
{
  m_velocity = velocity;
}

void ExtendedPose2d::setOrientation(double yaw)
{
  PoseTraits<Pose2d>::fromOrientationRPY(m_pose,
                                         yaw);
}

void ExtendedPose2d::setOrientation(const Eigen::Matrix2d& rotation_matrix)
{
  m_pose.linear() = rotation_matrix;
}

Pose2d& ExtendedPose2d::pose()
{
  return m_pose;
}

void ExtendedPose2d::setPose(double x, double y, double yaw)
{
  PoseTraits<Pose2d>::fromPositionAndOrientationRPY(m_pose, x, y, yaw);
}

void ExtendedPose2d::setPose(const Pose2d& pose)
{
  m_pose = pose;
}

void ExtendedPose2d::print(std::ostream& out) const
{
  out << std::fixed << "("
      << getX() << ", "
      << getY() << " | "
      << getYaw()
      << ")" << std::endl;
}

void ExtendedPose2d::print(std::ostream& out, const std::string& comment) const
{
  out << comment << " ";
  print(out);
}

void ExtendedPose2d::directTo(const Position2d& dest_position)
{
  const Eigen::Vector2d direction = dest_position - this->getPosition();
  this->setOrientation(std::atan2(direction.y(), direction.x()));
}

double ExtendedPose2d::distance(const ExtendedPose2d& other) const
{
  double transl_diff = (m_pose.translation()-other.m_pose.translation()).norm();
  double yaw_diff = std::abs(this->getYaw() - other.getYaw());
  if (yaw_diff > M_PI)
  {
    yaw_diff -= M_PI;
  }
  return (2.*transl_diff + yaw_diff) / 3.;
}

std::ostream& operator << (std::ostream &os, const ExtendedPose2d &pose)
{
  pose.print(os);
  return os;
}

} // end of ns
} // end of ns

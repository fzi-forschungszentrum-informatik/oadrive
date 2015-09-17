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
 * \date    2015-02-06
 *
 */
//----------------------------------------------------------------------
#include "Trajectory2d.h"
#include <algorithm>
#include <fstream>
#include <iostream>

namespace oadrive {
namespace core {

Trajectory2d::~Trajectory2d()
{
}

bool& Trajectory2d::curvatureAvailable()
{
  return m_curvature_available;
}

const bool& Trajectory2d::curvatureAvailable() const
{
  return m_curvature_available;
}

bool& Trajectory2d::velocityAvailable()
{
  return m_velocity_available;
}

const bool& Trajectory2d::velocityAvailable() const
{
  return m_velocity_available;
}

Pose2d& Trajectory2d::operator ()(std::size_t index)
{
  return m_trajectory[index].pose();
}

const Pose2d& Trajectory2d::operator ()(std::size_t index) const
{
  return m_trajectory[index].pose();
}

bool& Trajectory2d::isForwardTrajectory()
{
  return m_is_forward_trajectory;
}

const bool& Trajectory2d::isForwardTrajectory() const
{
  return m_is_forward_trajectory;
}

double Trajectory2d::calculateCurvature(const Position2d& p1,
                                        const Position2d& p2,
                                        const Position2d& p3,
                                        bool backward) const
{
  //      |b-a| |c-a| |b-c|
  // r = ------------------
  //       | bx-ax  by-ay |
  //     2 | cx-ax  cy-ay |
  const double numerator = std::sqrt((p2-p1).squaredNorm()
                                     * (p3-p1).squaredNorm()
                                     * (p2-p3).squaredNorm());
  const double denominator = 2.0 * (((p2.x()-p1.x())*(p3.y()-p1.y()))
                                    - ((p3.x()-p1.x())*(p2.y()-p1.y())));
  const double curvature = denominator / numerator;
  return backward ? -curvature : curvature;
}

void Trajectory2d::calculateCurvature()
{
  if (m_trajectory.size() < 3) // trajectories with 1 or 2 points have curvature of 0.
  {
    for (std::size_t i=0; i<m_trajectory.size(); ++i)
    {
      m_trajectory[i].setCurvature(0.);
    }
    m_curvature_available = true;
    return;
  }

  // iterate over all but the first and the last entries
  for (std::size_t i=1; i< m_trajectory.size()-1; ++i)
  {
    m_trajectory[i].setCurvature(
      calculateCurvature(m_trajectory[i-1].getPosition(),
                           m_trajectory[i].getPosition(),
                           m_trajectory[i+1].getPosition(),
                           !m_is_forward_trajectory));
  }

  // The first entry will have the same curvature as the second
  m_trajectory[0].setCurvature(m_trajectory[1].getCurvature());

  // The last entry will have the same curvature as the second last
  m_trajectory[m_trajectory.size()-1].setCurvature(m_trajectory[m_trajectory.size()-2].getCurvature());

  // change flag to signal the availability of curvature data
  m_curvature_available = true;
}

double Trajectory2d::length() const
{
  double length = 0.;
  for (std::size_t i=0; i<m_trajectory.size()-1; ++i)
  {
    length += (m_trajectory[i+1].getPosition() - m_trajectory[i].getPosition()).norm();
  }
  return length;
}

double Trajectory2d::lengthUpTo(std::size_t index) const
{
  double length = 0.;
  for (std::size_t i=0; i<index; ++i)
  {
    length += (m_trajectory[i+1].getPosition() - m_trajectory[i].getPosition()).norm();
  }
  return length;
}

double Trajectory2d::lengthFrom(std::size_t index) const
{
  double length = 0.;
  for (std::size_t i=index; i<m_trajectory.size()-1; ++i)
  {
    length += (m_trajectory[i+1].getPosition() - m_trajectory[i].getPosition()).norm();
  }
  return length;
}

double Trajectory2d::lengthBetween(std::size_t start_index, std::size_t end_index) const
{
  double length = 0.;

  if (end_index < start_index)
  {
    std::swap(start_index, end_index);
  }

  for (std::size_t i=start_index; i<end_index; ++i)
  {
    length += (m_trajectory[i+1].getPosition() - m_trajectory[i].getPosition()).norm();
  }
  return length;
}

void Trajectory2d::calculateOrientations()
{
  assert((this->size() > 1) && "Trajectory must have at least 2 poses.");

  // all but the last
  for (std::size_t i=0; i<m_trajectory.size()-1; ++i)
  {
    m_trajectory[i].directTo(m_trajectory[i+1].getPosition());
    if (!isForwardTrajectory())
      m_trajectory[i].setYaw(m_trajectory[i].getYaw() + M_PI);
  }

  // the last is a linear extrapolation from the second last
  m_trajectory[m_trajectory.size()-1].setYaw(m_trajectory[m_trajectory.size()-2].getYaw());
}

void Trajectory2d::fillWithTestData(double sampling_rate,
                                    bool set_velocity,
                                    bool movement_direction_forward)
{
  m_trajectory.clear();
  ExtendedPose2d pose;

  isForwardTrajectory() = movement_direction_forward;
  velocityAvailable() = set_velocity;

  for (double x=-10.; x<50.; x+=sampling_rate)
  {
    double y = 0.05*sin(x-3) - 0.0001*pow(x, 3) + 1;
    double yaw = atan(-0.0003*pow(x, 2) + 0.05*cos(3-x));  // yaw is atan of 1st derivative
    pose.setPose(x, y, yaw);
    pose.setVelocity(2. + .1*x);

    if (!movement_direction_forward)
    {
      pose.setYaw(yaw + M_PI);
    }
    m_trajectory.push_back(pose);
  }

  calculateCurvature();
  if (set_velocity)
  {
    m_velocity_available = true;
  }
  else
  {
    m_velocity_available = false;
  }
}

void Trajectory2d::toGnuplot(const std::string& filename_without_suffix) const
{
  const std::string gnuplot_filename = filename_without_suffix + ".gpl";
  const std::string pose_filename = filename_without_suffix + ".gpldata";
  const std::string curvature_filename = filename_without_suffix + "_curve.gpldata";

  // -- data formatting --

  std::ofstream pose_file(pose_filename.c_str());

  pose_file << "# Trajectory data set containing "<< m_trajectory.size() << " sets of position and orientation information." << std::endl;
  pose_file << "# To draw the poses the orientation is coded as two vectors in gnuplot from-delta style." << std::endl;
  pose_file << "# from: x  y  | delta: x  y" << std::endl;
  pose_file << "# "<< std::endl;
  pose_file << "# "<< std::endl;
  pose_file << "# The trajectory is for "
            << (this->isForwardTrajectory() ? "FORWARD" : "BACKWARD")
            << " motion." << std::endl;


  for (std::size_t i=0; i<m_trajectory.size(); ++i)
  {
    const Position2d from(m_trajectory[i].getPosition());
    const Position2d delta = m_trajectory[i].getOrientation() * Position2d::UnitX() * 0.02;

    pose_file << from.x()  << "  " << from.y() << "  ";
    if (velocityAvailable())
    {
      pose_file << m_trajectory[i].getVelocity() << "  ";
    }
    pose_file << delta.x() << "  " << delta.y() << "  ";
    if (velocityAvailable())
    {
      pose_file << 0.;
    }
    pose_file << std::endl;
  }

  pose_file << std::endl;

  // curvature plots
  if (curvatureAvailable())
  {
    std::ofstream curve_file(curvature_filename.c_str());

    for (std::size_t i=0; i<m_trajectory.size(); ++i)
    {
      // start for curvature lines
      const Position2d from(m_trajectory[i].getPosition());
      // end point for curvature lines
      const double curvature = m_trajectory[i].getCurvature() * 0.5; // apply scale for better visualization
      const double yaw = m_trajectory[i].getYaw();
      Position2d to;
      to.x() = from.x() - std::sin(yaw) * curvature;
      to.y() = from.y() + std::cos(yaw) * curvature;

      curve_file << from.x() << "  " << from.y();
      if (velocityAvailable())
      {
        curve_file << "  0.";
      }
      curve_file << std::endl;
      curve_file << to.x()   << "  " << to.y();
      if (velocityAvailable())
      {
        curve_file << "  0.";
      }
      curve_file << std::endl << std::endl << std::endl; // apply empty line to separate plot
    }
    curve_file.close();
  }

  pose_file.close();

  // -- the plot itself --

  std::ofstream gnuplot_file(gnuplot_filename.c_str(), std::ios::out);

  std::string plot_type = "plot";
  velocityAvailable() ? plot_type = "splot" : plot_type = "plot";

  gnuplot_file << "# Gnuplot file. Draw with gnuplot -p "<< pose_filename << std::endl;
  gnuplot_file << "set mapping cartesian" << std::endl;
  gnuplot_file << "set mouse" << std::endl;
  gnuplot_file << "set size ratio -1" << std::endl;
  gnuplot_file << plot_type + " '"<< pose_filename << "' using 1:2:3:4";
  if (velocityAvailable())
  {
    gnuplot_file << ":5:6";
  }
  gnuplot_file << " with vector";
  if (curvatureAvailable())
  {
    gnuplot_file << ", " << "'" << curvature_filename << "' using 1:2";
    if (velocityAvailable())
    {
      gnuplot_file << ":3";
    }
    gnuplot_file << " w l";
  }
  gnuplot_file << std::endl;
  gnuplot_file.close();
  std::cout << "GnuPlot file written to " << gnuplot_filename << std::endl;
}

void Trajectory2d::append(const Trajectory2d& other)
{
  for (std::size_t i=0; i<other.size(); ++i)
  {
    m_trajectory.push_back(other[i]);
  }
}

void Trajectory2d::shortcut()
{
  if (m_trajectory.empty())
  {
    return;
  }

  /* Transform each 'following' pose into the cs
   * of the 'current' pose. A violation to the desired
   * driving direction is found if
   *    x-value <= 0 and driving forward
   *    x-value >= 0 and driving backward
   */
  Pose2d temp;
  for (ContainerType::iterator current=m_trajectory.begin(),
       next=current+1;
       (current!=m_trajectory.end() && next!=m_trajectory.end());
       ++current, ++next)
  {
    temp = current->pose().inverse() * next->pose();
    if (((m_is_forward_trajectory && (temp.translation().x() <=0.)))
        || ((!m_is_forward_trajectory) && (temp.translation().x() >=0.)))
    {
      erase(next);
      --current; // step back to check the same place again
      next = current+1;
    }
  }

  // recalculate curvature.
  if (curvatureAvailable())
  {
    calculateCurvature();
  }
}

} // end of ns
} // end of ns

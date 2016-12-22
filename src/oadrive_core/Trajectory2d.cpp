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
 * \author  Sebastian Klemm <klemm@fzi.de>
 * \date    2015-02-06
 *
 */
//----------------------------------------------------------------------
#include "Trajectory2d.h"
#include "Interpolator.h"
#include "ExtendedPose2d.h"
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
  if(m_trajectory.size()== 0)
  {
    return 0;
  }
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

Trajectory2d Trajectory2d::simplify(double epsilon)
{
    Trajectory2d trajectory;
    trajectory.push_back(m_trajectory[0]);
    ramerDouglasPeuckerRecursion(trajectory, epsilon*epsilon, 0, m_trajectory.size()-1);

    return trajectory;
}

void Trajectory2d::ramerDouglasPeuckerRecursion(Trajectory2d& trajectory, double epsilonSquared, size_t begin, size_t end) {
	double dMax = 0;
  size_t index = 0;
	Position2d beginPosition = m_trajectory[begin].getPosition();
	Position2d endPosition = m_trajectory[end].getPosition();
	// calculate maximum distance to line of all points between points corresponding to begin and end position
	for(size_t i = begin+1; i < end; i++) {
		double dist = squaredLineDistance(beginPosition, endPosition, m_trajectory[i].getPosition());
		if(dist > dMax) {
			dMax = dist;
			index = i;
		}
	}

	if(dMax > epsilonSquared) {
		ramerDouglasPeuckerRecursion(trajectory, epsilonSquared, begin, index);
		ramerDouglasPeuckerRecursion(trajectory, epsilonSquared, index, end);
	}
	else {
		trajectory.push_back(m_trajectory[end]);
	}
}

double Trajectory2d::squaredLineDistance(const Position2d &p1, const Position2d &p2, const Position2d &p3)
{
    //inspired by the pSimpl lib http://psimpl.sourceforge.net/
    //make a vector of the line
    Position2d line = p2-p1;
    //make a vector to the test point from p1
    Position2d testPoint = p3-p1;
    //projection on the line
    double dotLineTestPoint = line.x()*testPoint.x()+line.y()*testPoint.y();
    //line is before p1
    if(dotLineTestPoint<=0 )
    {
        return (p1.x()-p3.x())*(p1.x()-p3.x())+(p1.y()-p3.y())*(p1.y()-p3.y());
    }
    //calculate squared length
    double lengthLine = line.x()*line.x()+line.y()*line.y();
   // dotLineTestPoint/lengthLine = relative point between p1 and p2
    //projection is outside the line defined by p1 and p2 (after p2)
    if(lengthLine<dotLineTestPoint)
    {
        //return distance to p2
        return (p2.x()-p3.x())*(p2.x()-p3.x())+(p2.y()-p3.y())*(p2.y()-p3.y());
    }
    //the projection is between p1 and p2
    //calculate projection point
    Position2d projectionPoint = (line*(dotLineTestPoint/lengthLine));
    //return distance between projection point and test point
    return (projectionPoint.x()-testPoint.x())*(projectionPoint.x()-testPoint.x())+(projectionPoint.y()-testPoint.y())*(projectionPoint.y()-testPoint.y());


}

std::ostream& operator<< (std::ostream &os, const Trajectory2d &traj)
{
	const std::streamsize oldPrecision = os.precision();
	os.precision( 20 );
	os << "Trajectory2d: (" << traj.size() << " points)" << std::endl;
	os << "\tforward: " << traj.isForwardTrajectory() << std::endl;
	for( size_t i = 0; i < traj.size(); i++ )
		os << "\t" << traj[i];
	os.precision( oldPrecision );
	return os;
}
std::istream& operator>> (std::istream &is, Trajectory2d &traj)
{
	is.ignore(15);		// Ignore "Trajectory2d: ("
	unsigned int size = 0;
	is >> size;
	is.ignore(9);		// ignore the rest of the line
	is.ignore(10);		// ignore "\tforward: "
	bool isForward;
	is >> isForward;
	is.ignore(1);	// ignore newline
	ExtendedPose2d pose;
	for( unsigned int i = 0; i < size; i++ )
	{
		is.ignore(1);	// ignore tab ("\t")
		is >> pose;
		traj.push_back( pose );
		is.ignore(1);	// ignore newline
	}
	traj.isForwardTrajectory() = isForward;
	return is;
}

bool Trajectory2d::poseAtDistanceFrom(std::size_t index, double distance,
                                    ExtendedPose2d& pose) const
{
  // --- Catch all invalid cases ---

  // index out of bounds
  if ((index >= m_trajectory.size()))
  {
    // std::cerr << "Trajectory::poseAtDistanceFrom(): Index out of bounds." << std::endl;
    return false;
  }

  // fist pose and negative distance
  if ((index == 0) && (distance < 0))
  {
    // std::cerr << "Trajectory::poseAtDistanceFrom(): Using first pose as start pose, but got a negative distance." << std::endl;
    return false;
  }

  // last pose and positive distance
  if ((index == m_trajectory.size()-1) && (distance >0))
  {
    // std::cerr << "Trajectory::poseAtDistanceFrom(): Using last pose as start pose, but got a positive distance." << std::endl;
    return false;
  }


  // easiest case, no need to calculate anything...
  if (distance == 0.)
  {
    pose = m_trajectory.at(index);

    return true;
  }

  // --- Check if using distance would cause to walk over trajectory's bounds ---
  if (distance > 0.) // walk forward
  {
    //    std::cout << " --- walk forward ---" << std::endl;
    if (distance > lengthBetween(index, m_trajectory.size()-1))
    {
      //       std::cerr << "Trajectory::poseAtDistanceFrom(): Cannot walk over end of trajectory" << std::endl;
      return false;
    }

    for (std::size_t i=index; i<m_trajectory.size()-1; ++i)
    {
      const double length_between = lengthBetween(i, i+1);
      //      std::cout << " --- length between (" << i << ", " << i+1 << ") = " << length_between << std::endl;
      if (distance > length_between)
      {
        /* shorten the remaining distance until
         * it is smaller than the length between this and the next pose */
        distance -= length_between;
        //        std::cout << ".. shortening" << std::endl;
        //        std::cout << " remaining distance: " << distance << std::endl;
      }
      else
      {
        //        std::cout << ".. interpolating" << std::endl;
        const double interpolation_ratio = distance / length_between;
        pose = Interpolator::interpolateLinear(m_trajectory.at(i),
                                                         m_trajectory.at(i+1),
                                                         interpolation_ratio);
        return true;
      }
    }
  }
  else // walk backward
  {
//        std::cout << " --- walk backward ---" << std::endl;
    distance *= -1;

    if (distance > lengthBetween(index, 0))
    {
//            std::cerr << "Trajectory::poseAtDistanceFrom(): Cannot walk over start of trajectory" << std::endl;
//            std::cerr << "Trajectory::poseAtDistanceFrom(): Distance: " << distance << " available length between indices ("<< index << ", 0): " << lengthBetween(index, 0) << std::endl;
      return false;
    }

//        std::cout << " --- index = " << index << std::endl;
    for (std::size_t i=index; i>0; --i)
    {
      const double length_between = lengthBetween(i, i-1);
//            std::cout << " --- length between (" << i << ", " << i-1 << ") = " << length_between << std::endl;
      if (distance > length_between)
      {
        /* shorten the remaining distance until
         * it is smaller than the length between this and the next pose */
        distance -= length_between;
//                std::cout << ".. shortening" << std::endl;
//                std::cout << " remaining distance: " << distance << std::endl;
      }
      else
      {
//                std::cout << ".. interpolating" << std::endl;
        const double interpolation_ratio = distance / length_between;
        pose = Interpolator::interpolateLinear(m_trajectory.at(i),
                                                                m_trajectory.at(i-1),
                                                                interpolation_ratio);


        return true;
      }
    }
  }
  std::cerr << "Trajectory::poseAtDistanceFrom(): This message should never occur. Something went wrong!" << std::endl;
  return false;
}

//void Trajectory2d::interpolateLinearWithDistance(double min_distance_between_positions, bool adopt_velocity)
//{
//  if (!m_trajectory.size())
//  {
//    return; // An empty trajectory cannot be interpolated
//  }

//  ContainerType interpolated_trajectory;
//  double velocity = m_trajectory.at(0).getVelocity();

//  // The first pose is always valid
//  interpolated_trajectory.push_back(m_trajectory.at(0));

//  for (std::size_t i=0; i<m_trajectory.size()-1; ++i)
//  {
//    const double distance = (m_trajectory.at(i).getPosition() - m_trajectory.at(i+1).getPosition()).norm();
//    if (distance > min_distance_between_positions)
//    {
//      double remaining_distance = distance; // initialization
//      std::size_t counter = 1;
//      while (remaining_distance > min_distance_between_positions)
//      {
//        const double interpolation_ratio = counter * min_distance_between_positions / distance;
//        // std::cout << "  interpolation_ratio is now = " << interpolation_ratio << std::endl;
//        ExtendedPose2d interpolated_pose(Interpolator::interpolateLinear(m_trajectory.at(i).pose(),
//                                                                              m_trajectory.at(i+1).pose(),
//                                                                              interpolation_ratio));

//        interpolated_pose.setYaw(m_trajectory.at(i).getYaw());
//        if (adopt_velocity && m_velocity_available)
//        {
//          interpolated_pose.setVelocity(velocity);
//        }
//        if (!adopt_velocity && m_velocity_available)
//        {
//          interpolated_pose.setVelocity(Interpolator::interpolateLinear(m_trajectory.at(i).getVelocity(),
//                                                                                  m_trajectory.at(i+1).getVelocity(),
//                                                                                  interpolation_ratio));
//        }

//        // interpolated_pose.get().print(std::cout,"Interpolated pose:");

//        interpolated_trajectory.push_back(interpolated_pose);
//        remaining_distance -= min_distance_between_positions;
//        ++counter;
//      }
//    }

//    // We do not remove any poses here, so we add the next pose no matter how near or far it is
//    interpolated_trajectory.push_back(m_trajectory.at(i+1));
//  }

//  // store new trajectory
//  std::swap(m_trajectory, interpolated_trajectory);
//  this->calculateCurvature();
//}

void Trajectory2d::resample(const double sampling_distance)
{
  if (!resampleSanityCheck(sampling_distance))
  {
    return;
  }

  // --- Resampling ---
  Trajectory2d resampled_trajectory;
  ExtendedPose2d pose;
//  pose.header() = m_trajectory.front().header();

  double travelled_distance = sampling_distance; // initialization
  const double length = this->length();

  // Add the first original trajectory pose
  resampled_trajectory.push_back(m_trajectory.at(0));

  // -- Travel along the trajectory, resample where necessary. --
  while (travelled_distance < length) // Walk over the trajectory once
  {
    //    std::cout << "have not reached the end yet" << std::endl;
    if (!poseAtDistanceFrom(0, travelled_distance, pose))
    {
      std::cerr << "Trajectory::resample(): Something went wrong when calling poseAtDistanceFrom(). Will abort here!" << std::endl;
      return;
    }
    resampled_trajectory.push_back(pose);
    // resampled_trajectory.print(std::cout, "wip: resampled trajectory is now:");

    if (travelled_distance < length) // to make sure travelled_distance remains valid for use after while loop
    {
      travelled_distance += sampling_distance;
    }
  }

  // If not already happened add the last original trajectory pose
  if (travelled_distance != length)
  {
    resampled_trajectory.push_back(m_trajectory.at(m_trajectory.size()-1));
  }

  // Store new trajectory contents
  m_trajectory = resampled_trajectory.m_trajectory;

  m_curvature_available = false;

  // this->print(std::cout, "After resampling trajectory is now:");
}

bool Trajectory2d::resampleSanityCheck(const double sampling_distance)
{
  assert(sampling_distance > 0 && "Trajectory::resampleSanityCheck(): The given sampling distance must be >0!");
  if (sampling_distance <= 0.)
  {
    return false;
  }

  if (!m_trajectory.size())
  {
    //std::cerr << "Trajectory::resampleSanityCheck(): Trajectory is empty, won't do anything." << std::endl;
    return false;
  }

  return true;
}

} // end of ns
} // end of ns

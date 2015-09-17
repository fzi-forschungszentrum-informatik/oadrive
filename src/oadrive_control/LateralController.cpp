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
 * \author  Raphael Frisch <frisch@fzi.de>
 * \date    2014-11-15
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-01-08
 *
 */
//----------------------------------------------------------------------
#include "LateralController.h"

#include <cmath>
#include <oadrive_core/Interpolator.h>

using namespace oadrive::core;

namespace oadrive {
namespace control {

LateralController::LateralController()
  : BEFORE_ATAN(0.694271),
    AFTER_ATAN(0.9),
    KD(1.0),
    //REF_POINT_1(1.0),
    //REF_POINT_2(2.0),
    SIGN_FCT_LIMIT(0.7),
    WHEELBASE(0.36),
    WEIGHTING_DISTANCE(0.62),
    MAX_FUNCTION(70),
    MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER(20),
    REACHED_ZONE_DISTANCE(20),
    STRAIGHT(Position2d(1.0, 0.0)),
    m_reached(true)
{
  m_trajectory.isForwardTrajectory() = true;
}

float LateralController::calculateSteering(const Pose2d &vehicle_pose)
{
  assert(m_trajectory.size() >= MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER);
  assert(m_trajectory.curvatureAvailable());

  updatePosAndCheckForReached(vehicle_pose);

  Controller(PoseTraits<Pose2d>::yaw(vehicle_pose), m_projected.getYaw(), m_projected.getCurvature(), m_distance);

  return m_delta;
}

void LateralController::updatePosAndCheckForReached(const Pose2d& vehicle_pose)
{
  calculateProjection(m_trajectory, vehicle_pose.translation(), m_projected, m_distance, m_nearest_point_index);

  // check if at end of trajectory
  const bool position_good_enough = m_distance < 0.5 && (m_projected.getYaw() - PoseTraits<Pose2d>::yaw(vehicle_pose)) < M_PI/3;
  if (position_good_enough && (m_projected.getPosition() - m_trajectory.back().getPosition()).norm() < REACHED_ZONE_DISTANCE)
  {
    m_reached = true;
  }

  // check custom position
  checkReachedPoints();
}

bool LateralController::calculateProjection(const Trajectory2d& trajectory, const Position2d& position, ExtendedPose2d& projection,
                                            double& distance, std::size_t& nearest_pose_index) const
{
  float dist1, dist2, ratio, curr_distance_squared;
  float t;

  float shortest_distance_squared = std::numeric_limits<float>::infinity();

  bool shortest_distance_found_before_first_point = true;
  nearest_pose_index = 0;

  const double& xP = position.x();
  const double& yP = position.y();
  for( u_int32_t i = 0; i < trajectory.size()-1 ; ++i)
  {
    const double& x1 = trajectory[i].getX();
    const double& y1 = trajectory[i].getY();
    const double& x2 = trajectory[i+1].getX();
    const double& y2 = trajectory[i+1].getY();

    _position_type Xnew, Ynew;

     const double& APx = xP - x1;
     const double& APy = yP - y1;
     const double& ABx = x2 - x1;
     const double& ABy = y2 - y1;
     const double& magAB2 = ABx*ABx + ABy*ABy;
     const double& ABdotAP = ABx*APx + ABy*APy;
     t = ABdotAP / magAB2;

     if ( t < 0)
     {
       Xnew = x1;
       Ynew = y1;
     }
     else if (t > 1)
     {
       Xnew = x2;
       Ynew = y2;
     }
    else
    {
      Xnew = x1 + ABx*t;
      Ynew = y1 + ABy*t;
    }


    _position_type a = (xP - Xnew)*(xP - Xnew);
    _position_type b = (yP - Ynew)*(yP - Ynew);
    curr_distance_squared = (a + b);

    if (curr_distance_squared < shortest_distance_squared)
    {
      shortest_distance_squared = curr_distance_squared;
      shortest_distance_found_before_first_point = (i == 0 && t < 0);

      nearest_pose_index = i;

      dist1 = sqrt((Xnew - x1)*(Xnew - x1) + (Ynew - y1)*(Ynew - y1));
      dist2 = sqrt((Xnew - x2)*(Xnew - x2) + (Ynew - y2)*(Ynew - y2));
      ratio = dist1/(dist1+dist2);

    }
  }

  // A-B:
  const Eigen::Vector2d vector_ab = trajectory[nearest_pose_index].getPose().rotation() * STRAIGHT;
  // A-Vehicle
  const Eigen::Vector2d vector_a_vehicle = position - trajectory[nearest_pose_index].getPosition();

  // pose of the car is before the beginning of the trajectory -> use distance to projection on AB-Vector
  if (shortest_distance_found_before_first_point)
  {
    // copy speed and curvature from first point
    projection = trajectory.front();

    // Projection
    const double t = vector_a_vehicle.dot(vector_ab)/vector_ab.squaredNorm();
    projection.setPosition(trajectory[0].getPosition() + vector_ab * t);

    // Calculate distance
    distance = (projection.getPosition() - position).norm();

  }
  else
  {
    //calculate projected pose
    projection = oadrive::core::Interpolator::interpolateLinear(trajectory[nearest_pose_index], trajectory[nearest_pose_index+1], ratio);

    // distance calculation
    distance = sqrt(shortest_distance_squared);

  }



  //Sign of distance
  Eigen::Matrix<double, 2, 2> m;
  m.col(0) <<vector_ab;
  m.col(1) <<vector_a_vehicle;

  // If the determinant is negative, the point lies on the right hand to the line
  if (m.determinant() > 0.0)
    distance = -distance;

  return shortest_distance_found_before_first_point;
}

void LateralController::Controller(double psiArg, double thetaArg, double kappaArg, double distanceArg)
{
  // Vorsteuerung
  float vorsteuerung = (float)-(((AFTER_ATAN * atan(BEFORE_ATAN*WHEELBASE*kappaArg))*180)/M_PI);

  // Winkel
  float angleFinal   = -KD * NormalizeAngle(thetaArg - psiArg);
  if (!m_trajectory.isForwardTrajectory())
    angleFinal = -angleFinal;
	
  // Distanz
  float distFinal    = -/*AdaptRefPointFuction(velocityArg) * */ WEIGHTING_DISTANCE * SignedFunction(distanceArg);


  m_delta = vorsteuerung + angleFinal + distFinal;

  if(m_delta > 100)
    m_delta = 100;
  else if(m_delta < -100)
    m_delta = -100;
}

float LateralController::NormalizeAngle(float angleArg)
{
  while (std::abs(angleArg) > M_PI)
  {
    angleArg = (angleArg >= 0) ? angleArg - 2*M_PI : angleArg + 2*M_PI;
  }

  angleArg = angleArg * 100 / M_PI;
  return angleArg;
}

float LateralController::SignedFunction(float numArg)
{
  float res = numArg/SIGN_FCT_LIMIT*MAX_FUNCTION;
  if (res > MAX_FUNCTION)
    res = MAX_FUNCTION;
  else if (res < -MAX_FUNCTION)
    res = -MAX_FUNCTION;

  return res;
}

void LateralController::checkReachedPoints()
{
  for (u_int8_t i = 0; i < RP_COUNT; i++)
  {
    if (!m_reached_positions[i].reached)
    {
      if ((m_projected.getPosition() - m_reached_positions[i].position).norm() < m_reached_positions[i].reached_distance)
        m_reached_positions[i].reached = true;
    }
  }
}

/*float LateralController::AdaptRefPointFuction(float numArg)
{

//TODO for calibration
return 1;
//return numArg;
  float res = numArg;
  if(abs (numArg) > REF_POINT_2){
    res = 0;
  }
  if((abs (numArg) > REF_POINT_1) && (abs(numArg) < REF_POINT_2)){
    res = (1-((abs(numArg)-REF_POINT_1)/(REF_POINT_2-REF_POINT_1))) * abs(numArg);
  }
  if(abs(numArg) < REF_POINT_1) res = 1;
    return res;
}*/

}
}// ns

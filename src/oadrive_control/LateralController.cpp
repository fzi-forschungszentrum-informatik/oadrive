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
#include <oadrive_control/controlLogging.h>
#include <iostream>
#include <oadrive_util/Config.h>
using namespace oadrive::core;
using icl_core::logging::endl;

namespace oadrive {
namespace control {

LateralController::LateralController():
  m_BeforeAtan(1),
  m_AfterAtan(1*1.3369),		//measuered
  mKD(1.0),
  SIGN_FCT_LIMIT(0.7),
  WHEELBASE(0.36),
  mWeightingDistance(2),
  mMaxFunction(30),
  MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER(3),
  REACHED_ZONE_DISTANCE(0.35),
  mVorsteuerungA(25.542),
  mVorsteuerungB(1.089),
  STRAIGHT(Position2d(1.0, 0.0)),
  m_distancePID(0),
  m_reached(true),
  m_KI(10),
  m_ISumMax(2),
  m_ITA(0.1),
  m_PControl(200)
{
  m_trajectory.isForwardTrajectory() = true;
  //Read config from config File. If the config file is not available it will take the default value

  //Weighting the angle (If the angle of the traj and of the car doesn't fit, how strong should it steer?)
  mKD = (float)oadrive::util::Config::getDouble("Driver","KD",1.0);
  //Weighting distance. (If the car is not on the traj. how strong should it steering back)
  //don't make this value to high (stability!)
  mWeightingDistance = (float)oadrive::util::Config::getDouble("Driver","WeightingDistance",2.0);
  mMaxFunction = (float)oadrive::util::Config::getDouble("Driver","MaxFunction",30.0);
  //Precontrol. Measure 3 or 4 circles an make a leastsquare aproximation steeringValue = 1/r*A+B
  //This is very important for this controller
  mVorsteuerungA = (float)oadrive::util::Config::getDouble("Driver","VorsteuerungA",25.542);
  mVorsteuerungB = (float)oadrive::util::Config::getDouble("Driver","VorsteuerungB",1.089);
  m_BeforeAtan = (float)oadrive::util::Config::getDouble("precontrol","beforeAtan", 1.238);
  m_AfterAtan = (float)oadrive::util::Config::getDouble("precontrol","AfterAtan", 1.095);
  m_OffsetAtan = (float)oadrive::util::Config::getDouble("precontrol", "OffsetAtan", 0.02704);
  std::cout<<"Vorsteuerung: "<<mVorsteuerungA<<std::endl;
}

float LateralController::calculateSteering(const Pose2d &vehicle_pose)
{
  assert(m_trajectory.size() >= MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER);
  assert(m_trajectory.curvatureAvailable());
  updatePosAndCheckForReached(vehicle_pose);
  double curvature;
  if(m_nearest_point_index+2 < m_trajectory.size())
  {
    curvature = m_trajectory[m_nearest_point_index+2].getCurvature();
  }
  else
  {
    curvature = m_projected.getCurvature();
  }
  Controller(PoseTraits<Pose2d>::yaw(vehicle_pose), m_projected.getYaw(), curvature, m_distance);

  return m_delta;
}

void LateralController::updatePosAndCheckForReached(const Pose2d& vehicle_pose)
{
  calculateProjection(m_trajectory, vehicle_pose.translation(), m_projected, m_distance, m_nearest_point_index);

  // look for angle in region (-M_PI .. M_PI)
  float angleDifference = m_projected.getYaw() - PoseTraits<Pose2d>::yaw(vehicle_pose);
  if(angleDifference > M_PI) {
    angleDifference = 2 * M_PI - angleDifference;
  }
  else if(angleDifference < -M_PI) {
    angleDifference = 2 * M_PI + angleDifference;
  }

  // check if at end of trajectory
  const bool position_good_enough = m_distance < 0.5 && std::abs(angleDifference) < M_PI/3;
/*  LOGGING_INFO( latLogger, "position_good_enough: " << position_good_enough << endl
                          << "m_distance: " << m_distance << endl
                          << "m_projected.getYaw(): " << m_projected.getYaw() << endl
                          << "PoseTraits<Pose2d>::yaw(vehicle_pose): " << PoseTraits<Pose2d>::yaw(vehicle_pose) << endl
                          << "old difference last two: " << (m_projected.getYaw() - PoseTraits<Pose2d>::yaw(vehicle_pose)) << endl
                          << "hopefully corrected difference: " << angleDifference << endl);*/

  //do some more angle magic
  ExtendedPose2d lastPose = m_trajectory.back();
  ExtendedPose2d sndLastPose = m_trajectory[m_trajectory.size()-2];
  ExtendedPose2d vecPose = ExtendedPose2d(vehicle_pose);
  
  
  double x1 = sndLastPose.getX() - lastPose.getX();
  double x2 = vecPose.getX() - lastPose.getX();

  double y1 = sndLastPose.getY() - lastPose.getY();
  double y2 = vecPose.getY() - lastPose.getY();

  double denom1 = std::max( sqrt( x1*x1 + y1*y1 ), 0.000001 );
  double denom2 = std::max( sqrt( x2*x2 + y2*y2 ), 0.000001 );
  x1 = x1 / denom1;
  y1 = y1 / denom1;
  x2 = x2 / denom2;
  y2 = y2 / denom2;
  double angle = acos(x1*x2 + y1*y2);

  bool behind = true;
  if(angle < M_PI/2){
    behind = false;
  }
  
/*  LOGGING_INFO( latLogger, "position_good_enough: " << position_good_enough << endl
                           << "difference: " << (m_projected.getPosition() - m_trajectory.back().getPosition()).norm() << endl
                            << "behind: " << behind << endl); */
  if (position_good_enough && (m_projected.getPosition() - m_trajectory.back().getPosition()).norm() < REACHED_ZONE_DISTANCE && behind)
  {
    m_reached = true;
  }

  // check custom position
  checkReachedPoints();
}

bool LateralController::calculateProjection(const Trajectory2d& trajectory, const Position2d& position, ExtendedPose2d& projection,
                                            double& distance, std::size_t& nearest_pose_index)
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
  m_ratio = ratio;

  // A-B:
  const Position2d vector_ab = trajectory[nearest_pose_index].getPose().rotation() * STRAIGHT;
  // A-Vehicle
  const Position2d vector_a_vehicle = position - trajectory[nearest_pose_index].getPosition();

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

void LateralController::controlDistance()
{
  m_ISum += m_distance;
  //look if mISum is to high
  if(m_ISum > m_ISumMax)
    m_ISum = m_ISumMax;
  else if(m_ISum < -m_ISumMax)
    m_ISum = -m_ISumMax;
  //PI Control
  //LOGGING_INFO(latLogger," m_Distance: "<<m_distance<<endl);
  m_distancePID = m_distance*m_PControl + m_KI*m_ITA*m_ISum;
}

void LateralController::Controller(double psiArg, double thetaArg, double kappaArg, double distanceArg)
{
  float vorsteuerung;
  // Vorsteuerung
  if(kappaArg != kappaArg)
  {
    //kappaArg is NAN that is very evil
    vorsteuerung = 0;
    //LOGGING_ERROR(latLogger,"KappaArg is NAN. Can't use feedforward control."<<endl);
  }
  else
  {
    //
//    vorsteuerung = -kappaArg*mVorsteuerungA+mVorsteuerungB;
    vorsteuerung = (float)-(((m_AfterAtan * atan(m_BeforeAtan*WHEELBASE*kappaArg) + m_OffsetAtan)*180)/M_PI);
  }

  // Winkel
  float angleFinal   = -mKD * NormalizeAngle(thetaArg - psiArg);
  if (!m_trajectory.isForwardTrajectory())
    angleFinal = -angleFinal;

  // Distanz
  float distFinal    = -/*AdaptRefPointFuction(velocityArg) * */ mWeightingDistance * SignedFunction(distanceArg);
  //  float distFinal = -m_distancePID;

  
  m_delta = vorsteuerung + angleFinal + distFinal;
  //LOGGING_INFO(latLogger,"[Controller] Vorsteuerung: "<<vorsteuerung<<" Angle Final: "<<angleFinal<<" distFinal: "<<distFinal<<"distance: "<<distanceArg<<"m_delta"<<m_delta<<endl);
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
//  signed_distance = std::min(signed_distance, p_signed_distance_max);
//  signed_distance = std::max(signed_distance, p_signed_distance_min);

  float res = numArg;
//  float res = numArg/SIGN_FCT_LIMIT*mMaxFunction;
  if (res > mMaxFunction)
    res = mMaxFunction;
  else if (res < -mMaxFunction)
    res = -mMaxFunction;

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

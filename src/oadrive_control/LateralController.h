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
 * \author  Raphael Frisch <frisch@fzi.de>
 * \date    2014-11-15
 *
 * \author  Christoph Rist <rist@fzi.de>
 * \date    2015-01-08
 *
 * \author  Robin Andlauer <andlauer@fzi.de>
 * \date    2017-06-20
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_CONTROL_LATERAL_CONTROLLER_H_INCLUDED
#define OADRIVE_CONTROL_LATERAL_CONTROLLER_H_INCLUDED

#include <vector>
#include <boost/array.hpp>

#include <oadrive_core/Trajectory2d.h>

namespace oadrive {
namespace control {

struct ReachedPosition
{
  oadrive::core::Position2d position;
  bool reached;
  double reached_distance;

  ReachedPosition()
  {
    reached = true;
  }
};

enum REACHED_POINT
{
  RP_PULL_OUT=0,
  RP_JUNCTION=1,
  RP_OVERTAKING=2,
  RP_STOPLINE=3,
  RP_COUNT=4
};

typedef boost::array<ReachedPosition, RP_COUNT> ReachedPositionArray;

class LateralController
{
public:
  //! Convenience pointer
  typedef boost::shared_ptr<LateralController> Ptr;

  typedef double _position_type;

  //! Constructor
  LateralController(float maxSteering);

  //! Destructor
  virtual ~LateralController()    {}

  //! Set trajectory
  void setTrajectory(const oadrive::core::Trajectory2d& trajectory)
  {
    m_trajectory = trajectory;
    m_reached = false;
  }
  
  /**
   * @brief calculateSteering
   * @param extended_trajectory
   * @param pose
   * @param reverse - set true if reversing
   * @return calculated steering value
   */
  float calculateSteering(const oadrive::core::Pose2d& vehicle_pose);

  bool calculateProjection(const oadrive::core::Trajectory2d& trajectory, const oadrive::core::Position2d& position,
                           oadrive::core::ExtendedPose2d& projection, double& distance, std::size_t& nearest_pose_index) ;

  //!
  bool hasReached() const      { return m_reached; }

  //! Read and write trajectory
  const oadrive::core::Trajectory2d& getTrajectory() const   { return m_trajectory; }
  oadrive::core::Trajectory2d& getTrajectory()               { return m_trajectory; }


  //! get projected pose
  const oadrive::core::ExtendedPose2d& getProjectedPose() const      { return m_projected; }

  //! get index on current trajectory
  size_t getIndexOfProjectedPose() const              { return m_nearest_point_index; }

  double getRatioToNextPoint() const          { return m_ratio; }

  void controlDistance();

  size_t getMinTrajPoints() { return MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER; }

private:

  //! Check reached points
  void checkReachedPoints();

  void updatePosAndCheckForReached(const oadrive::core::Pose2d &vehicle_pose);

  //!
  //! \brief Controller Calculates/controls steering angle to stay on trajectory.
  //! The P-controller calculates the angle by the sum of a precontrol, a weighted angle difference and a weighted distance.
  //! \param psiArg car orientation angle
  //! \param thetaArg orientation angle of the point on the trajectory that is closest to the car (car position projected on the trajectory)
  //! \param kappaArg curvature of the trajectory
  //! \param distanceArg distance between car and projected traj point
  //!
  void Controller(const double &psiArg, const double &thetaArg, const double &kappaArg, const double &distanceArg);

  //! Currently only caps the given input value (here: distance car<->traj)
  float SignedFunction(const float &numArg);

  //float AdaptRefPointFuction(float numArg);
  //! normalizes angle if it exceeds abs(pi)
  float NormalizeAngle(float angleArg);

  //! Current index of current pose on trajectory
  size_t m_nearest_point_index;

  //! Current trajectory
  oadrive::core::Trajectory2d m_trajectory;

  //! Current vehicle pose projected onto the traj
  oadrive::core::ExtendedPose2d m_projected;

  //! precontrol parameters
  float m_BeforeAtan;
  float m_AfterAtan;
  float m_OffsetAtan;
  //! Weighting the angle (If the angle of the traj and of the car doesn't fit, how strong should it steer?)
  float mKD;
  //! distance front wheel to back wheel in m
  const float WHEELBASE;
  //! Weighting distance. (If the car is not on the traj. how strong should it steering back)
  //! don't make this value to high (stability!)
  float mWeightingDistance;
  //! distance cap: if the car is too far away from the traj the distance weighting should be capped so the car does not steer back with 90° to the traj orientation
  //! (after traj is reached staying on the traj would be almost impossible if the car stands perpendicular to it)
  float mMaxFunction;
  const size_t MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER;
  const float REACHED_ZONE_DISTANCE;

  //!
  const oadrive::core::Position2d STRAIGHT;

  float m_delta;
  float m_direction;
  // current distance between car and projected traj point
  double m_distance;
  // ratio between 2 traj points where the projected traj point lies
  double m_ratio;

  bool m_reached;

  float mMaxSteering;

  //! Check for positions
  ReachedPositionArray m_reached_positions;


public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}
}// ns

#endif // OADRIVE_CONTROL_LATERAL_CONTROLLER_H_INCLUDED

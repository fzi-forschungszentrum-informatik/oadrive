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
  LateralController();

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

  void Controller(double psiArg, double thetaArg, double kappaArg, double distanceArg);

  float SignedFunction(float numArg);

  //float AdaptRefPointFuction(float numArg);

  float NormalizeAngle(float angleArg);

  //! Current index of current pose on trajectory
  size_t m_nearest_point_index;

  //! Current trajectory
  oadrive::core::Trajectory2d m_trajectory;

  //! Current vehicle pose projected onto m_trajectory
  oadrive::core::ExtendedPose2d m_projected;

  //!  properties
  float m_BeforeAtan;
  float m_AfterAtan;
  float m_OffsetAtan;
  float mKD;
  const float SIGN_FCT_LIMIT;
  const float WHEELBASE;
  float mWeightingDistance;
  float mMaxFunction;
  const size_t MIN_EXTENDED_POINTS_NEEDED_FOR_LATERAL_CONTROLLER;
  const float REACHED_ZONE_DISTANCE;
  //steering angle = a*(1/radius)+b this values must be measured. (drive with differnt steering angles and measure the radius)
  float mVorsteuerungA;
  float mVorsteuerungB;

  //!
  const oadrive::core::Position2d STRAIGHT;

  float m_delta;
  float m_direction;
  double m_distance;
  double m_distancePID;

  double m_ratio;

  bool m_reached;

  //store PI
  double m_KI;
  double m_ISum;
  const double m_ISumMax;
  const double m_ITA;
  const double m_PControl;

  //! Check for positions
  ReachedPositionArray m_reached_positions;


public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

}
}// ns

#endif // OADRIVE_CONTROL_LATERAL_CONTROLLER_H_INCLUDED

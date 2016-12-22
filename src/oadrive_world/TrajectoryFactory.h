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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-16
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_TRAJECTORYFACTORY_H
#define OADRIVE_WORLD_TRAJECTORYFACTORY_H

#include <oadrive_core/Trajectory2d.h>
#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_missioncontrol/MissionControlEnums.h>
#include "Patch.h"
#include <map>
#include "TrajectoryDatabase.h"
#include "MultiTrajectory.h"

#define GO_BACK_IN_TRAJ_HISTORY 5
#define GO_FRONT_PATCHES 10

#define ZICK_ZACK_ANGLE M_PI/2.0

namespace oadrive{
namespace world{

enum TrajectoryMode{ FIXED_TRAJECTORY, PATCH_TRAJECTORY };

class TrajectoryFactory
{
public:
  TrajectoryFactory();
  ~TrajectoryFactory() {};

  /*! Sets a fixed trajectory which will be used until the car arrives at its end.
   * From now on, getTrajectory will return this fixed trajectory. When its end has been
   * reached, an event should be triggered by the DriverModule to give the MissionControl a
   * chance to set a new trajectory.
   * This can be aborted by using one of the other modes, or by calling this function
   * again with another trajectory.
   * \note The factory might interpolate the trajectory.
   * \note If the trajectory is too short for the LateralController to handle, then it will
   * 		be extrapolated until the minimum number of points is reached. This extrapolation
   * 		happens before the beginning of the trajectory!
   * \param smooth If set to true, the function will smooth the trajectory before passing
   * 		the trajectory on to the Environment.*/
  //! \todo FIX THESE according to Eigen Guidelines!
  //void setFixedTrajectory( oadrive::core::Trajectory2d traj, bool smooth = false );
  void setFixedTrajectory( MultiTrajectory traj );

  /*! Selects trajectory depending on maneuver. */
  void setFixedTrajectory( enumTrajectory trajName, double scaleFactor = -1 );

  /*! Set to generateFromPatches mode.
   * generateFromPatches will be called repeatedly.*/
  void setGenerateFromPatches();

  /*! Generates a trajectory from the current street patches. */
  bool generateFromPatches(bool useOld = false);

  /*! Lets the TrajectoryFactory decide whether a new trajectory needs to be generated.
   * \note This will have no effect if setFixedTrajectory has been used (i.e. the
   * 		Factory is in PATCH_TRAJECTORY mode. */
  void requestUpdate(bool useOld = false);

  /*! Call to let the PatchTrajectory know where to start.
   * Should usually be set to the current car pose.
   * \note Needs to be called before the first call to generateFromPatches! */
  void setInitialPose( const ExtendedPose2d &initialPose );

  /*! Debug helper, generates test trajectories.
   * \param trajType "rectangle", "circle", "oval" */
  static Trajectory2d generateTestTrajectory( std::string trajType );

  /*! Removes back and forward zick zagging */
  //! \todo FIX THESE according to Eigen Guidelines!
  int removeZigZag( oadrive::core::Trajectory2d &traj );

  /*! Removes initial pose. */
  void reset();

  /*! Rotates the trajectory around the pose's yaw, then appends to the pose position.
   * \note This assumes that the trajectory starts at the origin. */
  static oadrive::core::Trajectory2d rotateAndMoveTrajectory(
      oadrive::core::Trajectory2d &traj,
      const oadrive::core::ExtendedPose2d &pose );

  /*! Rotates the trajectories around the pose's yaw, then appends to the pose position.
   * Rotates and moves all points in all the trajectories by the same pose.
   * \note This assumes that the trajectory starts at the origin. */
  static MultiTrajectory rotateAndMoveTrajectory(
      MultiTrajectory &multiTraj,
      const ExtendedPose2d &pose );

  /*! Moves the mutliTraj so that "pose" is the new origin:*/
  static MultiTrajectory setNewOrigin(
      MultiTrajectory &multiTraj,
      const oadrive::core::Position2d &pose );

  //!smoth trajectory
  //! \note accuracy will be lost due simplify step if nessesary up to 5cm
  static void smothTrajectory(oadrive::core::Trajectory2d &traj,double maxCurvature);
  //!check if trajectory radius is to high
  //!\return true if radius is ok and false if radius is to high
  static bool checkTrajectory(oadrive::core::Trajectory2d &traj, double maxCurvature);
  //! resample trajectory and smoth afterwards with a bSpline
  static void resampleAndInterpol(Trajectory2d &traj, double samplingDistance);

  void startDebugDumping( std::string folder );

  void clearOldTraj();

  void writeToFile( Trajectory2d& traj, std::string name );

private:

  /*! Prepends trajectory with more points towards the front (i.e. the start position) */
  void extrapolateFront( oadrive::core::Trajectory2d &traj );

  TrajectoryMode mTrajectoryMode;

  oadrive::core::ExtendedPose2d mInitialTrajPose;
  bool mInitialTrajPoseInitialized;

  oadrive::core::Trajectory2d mOldTraj;

  std::string mTrajectoryName;
  MultiTrajectory mCurrentMultiTraj;

  unsigned int mMultiCounter;
  double mMaxCurvature;

  std::string mDebugFolder;
  bool mDebugMode;
  unsigned int mTrajectoryCounter;

public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    bool isInFrontOnTraj(ExtendedPose2d &projection, size_t nearest_pose_index, Trajectory2d traj, size_t traj_index,
                         double offset);
};

}	// namespace
}	// namespace

#endif

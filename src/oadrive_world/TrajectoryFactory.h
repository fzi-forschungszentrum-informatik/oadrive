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
 * \author  David Zimmerer <dzimmerer@gmail.com>
 * \author  Micha Pfeiffer <ueczz@student.kit.edu>
 * \date    2016-01-16
 * 
 * \author  Simon Roesler <simon.roesler@student.kit.edu>
 * \date    2018
 * 
 * \author  Mark Hueneberg <hueneber@fzi.de>
 * \date    2018
 *
 */
//----------------------------------------------------------------------

#ifndef OADRIVE_WORLD_TRAJECTORYFACTORY_H
#define OADRIVE_WORLD_TRAJECTORYFACTORY_H

#include <oadrive_core/ExtendedPose2d.h>
#include <oadrive_core/Trajectory2d.h>
#include <map>
#include "MultiTrajectory.h"
#include "Patch.h"
#include "TrajectoryDatabase.h"

#define RAMP_START_OFFSET 0.45f
#define GO_BACK_IN_TRAJ_HISTORY 5
#define GO_FRONT_PATCHES 10

#define ZICK_ZACK_ANGLE M_PI / 2.0

#define DEBUG_VIEW0(pose) \
        double x0 = pose.getX(); \
        double y0 = pose.getY(); \
        double yaw0 = pose.getYaw(); \

#define DEBUG_VIEW1(pose) \
        double x1 = pose.getX(); \
        double y1 = pose.getY(); \
        double yaw1 = pose.getYaw(); \

namespace oadrive
{
namespace world
{

class Patch;
typedef boost::shared_ptr<Patch> PatchPtr;		// define before class because it uses the PatchPtr
typedef std::list<boost::shared_ptr<Patch> > PatchPtrList;
class Environment;
typedef boost::shared_ptr<Environment> EnvironmentPtr;

enum TrajectoryMode
{
  FIXED_TRAJECTORY, PATCH_TRAJECTORY
};

enum TurnDirection {
  STRAIGHT_TURN,
  LEFT_TURN,
  RIGHT_TURN
};

class TrajectoryFactory
{
public:
  // use a proper alignment when calling the constructor.
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajectoryFactory();

  ~TrajectoryFactory() {};

  /*! Selects trajectory depending on maneuver. */
  void setFixedTrajectory(enumTrajectory trajName, double scaleFactor = -1);

  /*! Set to generateFromPatches mode.
   * generateFromPatches will be called repeatedly.*/
  void setGenerateFromPatches();

  /*! Generates a trajectory from the current street patches. */
  core::Trajectory2d generateFromPatches(const Environment &env, bool useOld = false);

  core::Trajectory2d generatePulloutLeft();

  core::Trajectory2d generatePulloutRight();

  core::Trajectory2d generateBackup();
  MultiTrajectory generateRamp();
  core::Trajectory2d createCircle(double radius, double startAngle = 0.0);
  void moveTrajectoryToStart(core::Trajectory2d &traj, core::ExtendedPose2d start);
  void mirrorAtY(core::Trajectory2d &traj);

  MultiTrajectory generateCrossPark(core::Pose2d& takeoff);

  /*! Debug helper, generates test trajectories.
   * \param trajType "rectangle", "circle", "oval" */
  static core::Trajectory2d generateTestTrajectory(std::string trajType);

  /*! Returns the trajectory for a given direction (straight, left, right) to deal with a junction.
   * \param turnDirection TurnDirection.(STRAIGHT, LEFT, RIGHT) */
  static core::Trajectory2d getJunctionTrajectory(TurnDirection turnDirection, LaneType lane);

  static core::Trajectory2d getPreParkingTrajectory();

  static core::Trajectory2d getParkingTrajectory();

  void clearOldTraj();

private:

  /*! Rotates the trajectories around the pose's yaw, then appends to the pose
   * position. Rotates and moves all points in all the trajectories by the same
   * pose. \note This assumes that the trajectory starts at the origin. */
  MultiTrajectory rotateAndMoveTrajectory(MultiTrajectory &multiTraj,
                                          const core::ExtendedPose2d &pose);

  /*! Sets a fixed trajectory which will be used until the car arrives at its
   * end. From now on, getTrajectory will return this fixed trajectory. When its
   * end has been reached, an event should be triggered by the DriverModule to
   * give the MissionControl a chance to set a new trajectory. This can be
   * aborted by using one of the other modes, or by calling this function again
   * with another trajectory. \note The factory might interpolate the
   * trajectory. \note If the trajectory is too short for the LateralController
   * to handle, then it will be extrapolated until the minimum number of points
   * is reached. This extrapolation happens before the beginning of the
   * trajectory! \param smooth If set to true, the function will smooth the
   * trajectory before passing the trajectory on to the Environment.*/
  //! \todo FIX THESE according to Eigen Guidelines!
  // void setFixedTrajectory( oadrive::core::Trajectory2d traj, bool smooth =
  // false );
  void setFixedTrajectory(MultiTrajectory traj);

  bool isInFrontOnTraj(core::ExtendedPose2d &projection, size_t nearest_pose_index,
                       core::Trajectory2d traj, size_t traj_index, double offset);

  void writeToFile(core::Trajectory2d &traj, std::string name);

  void createTrajStartPoints(const Environment &env, core::Trajectory2d* newTrajectory,
                             PatchPtr outLastPastPatch);

  void createTrajFromPatches(core::Trajectory2d* newTrajectory, PatchPtr lastPastPatch);

  void validateTrajPoints(core::Trajectory2d* trajectory);

  void addTrajPointsForCrossSection(core::Trajectory2d* trajectory,
                                    const core::Trajectory2d &crossSectionTraj);

  void incorporateOldTraj(const Environment &env, core::Trajectory2d* trajectory, bool useOldTraj);

  /*!
   * Smooth the trajectory by using the current points to calculate B-Splines.
   * After that resample to get small curvatures and by doing so a smooth trajectory.
   * In case of a trejectory consisting of only to points the result is a linear path from
   * start to end.
   * @param trajectory which will be smoothed.
   */
  void smoothTrajectory(core::Trajectory2d* trajectory);

  /*! Removes back and forward zick zagging */
  //! \todo FIX THESE according to Eigen Guidelines!
  int removeZigZag(core::Trajectory2d &traj);

  TrajectoryMode mTrajectoryMode;

  std::string mTrajectoryName;

  core::Trajectory2d mOldTraj;
  MultiTrajectory mCurrentMultiTraj;

  std::string mDebugFolder;
  bool mDebugMode;
  unsigned int mTrajectoryCounter;

  core::ExtendedPose2d mlastCarPose;
  core::ExtendedPose2d mlastPatchPose;
  PatchPtr mPatchIterator = nullptr;

};

}  // namespace world
}  // namespace oadrive

#endif

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
 * \author  Jan Oberlaender <oberlaender@fzi.de>
 * \date    2013-03-25
 *
 * Some Pose Types using Eigen matrices for
 * - SE(2): position    (x, y) and orientation              (yaw)
 * - SE(3): position (x, y, z) and orientation (roll, pitch, yaw).
 *
 */
//----------------------------------------------------------------------
#ifndef OADRIVE_CORE_POSE_H_INCLUDED
#define OADRIVE_CORE_POSE_H_INCLUDED

#include <Eigen/Dense>
#include <oadrive_core/MathFunctions.h>

namespace oadrive {
namespace core {

/*! Data type Pose holds position and orientation information,
 *  e.g. for geometric calculations in navigation, calibration
 *  and kinematic context.
 */
template<typename T, int t_dimensions, int t_options = Eigen::AutoAlign>
struct Pose
{
  typedef typename Eigen::Transform<T, t_dimensions, Eigen::Isometry, t_options> type;
};


//----------------------------------------------------------------------
// Double-precision data types
//----------------------------------------------------------------------

//! Double-precision Pose in SE(2) with standard Eigen alignment.
typedef Pose<double, 2, Eigen::AutoAlign>::type Pose2d;

//! Double-precision Pose in SE(2) without Eigen alignment.
typedef Pose<double, 2, Eigen::DontAlign>::type UnalignedPose2d;

//! Double-precision Pose in SE(3) with standard Eigen alignment.
typedef Pose<double, 3, Eigen::AutoAlign>::type Pose3d;

//! Double-precision Pose in SE(3) without Eigen alignment.
typedef Pose<double, 3, Eigen::DontAlign>::type UnalignedPose3d;

//! Double-precision 2D Vector without alignment.
typedef Eigen::Matrix<double, 2, 1, Eigen::DontAlign> UnalignedVector2d;
//! Double-precision 3D Vector without alignment.
typedef Eigen::Matrix<double, 3, 1, Eigen::DontAlign> UnalignedVector3d;
//! Double-precision 4D Vector without alignment.
typedef Eigen::Matrix<double, 4, 1, Eigen::DontAlign> UnalignedVector4d;


//----------------------------------------------------------------------
// Pose traits
//----------------------------------------------------------------------

//! Pose traits for the generic case (default SE(3)).
template <typename T>
struct PoseTraits
{
  //! The underlying scalar datatype.
  typedef typename T::Scalar Scalar;

  //! Provides the number of dimensions at compile time.
  enum {
    SPATIAL_DIMENSIONS = T::Dim,
    TOTAL_DIMENSIONS   = T::Dim+T::Dim*(T::Dim-1)/2
  };

  //! Position vector type.
  typedef Eigen::Matrix<Scalar, SPATIAL_DIMENSIONS, 1> Position;

  //! Check the poses for equalitiy
  static bool isEqual(const T& pose_a, const T& pose_b)
  {
    return (pose_a.matrix() == pose_b.matrix());
  }

  /*! Returns the euler angles of the \a rot_matrix rotation matrix
   *  in yaw-pitch-roll convention in radians. This one is necessary,
   *  because the fcn eulerAngles normalizes to [0:pi]x[-pi:pi]x[-pi:pi],
   *  which is quite unuseful especially with regard to yaw.
   *  This code is from Eigen EulerAngles.h with a few changes.
   */
  static Eigen::Matrix<Scalar, 3, 1> yprAngles(const Eigen::Matrix<Scalar, 3, 3>& rot_matrix)
  {
    Eigen::Matrix<Scalar, 3, 1> res;
    res[0] = atan2(rot_matrix.coeff(1,0), rot_matrix.coeff(0,0));
    Scalar c2 = Eigen::Matrix<Scalar,2,1>(rot_matrix.coeff(2,2), rot_matrix.coeff(2,1)).norm();
    res[1] = atan2(-rot_matrix.coeff(2,0), c2);

    Scalar s1 = sin(res[0]);
    Scalar c1 = cos(res[0]);
    res[2] = atan2(s1*rot_matrix.coeff(0,2)-c1*rot_matrix.coeff(1,2), c1*rot_matrix.coeff(1,1) - s1 * rot_matrix.coeff(0,1));

    return res;
  }

  /*! Returns the orientation part of the \a pose in yaw-pitch-roll
   *  convention in radians.
   */
  static Eigen::Matrix<Scalar, 3, 1> ypr(const T& pose)
  {
    return yprAngles(pose.rotation());
  }

  /*! Returns the orientation part of the \a pose in roll-pitch-yaw
   *  convention in radians.
   */
  static Eigen::Matrix<Scalar, 3, 1> rpy(const T& pose)
  {
    Eigen::Matrix<Scalar, 3, 1> ea = yprAngles(pose.rotation());
    std::swap(ea(0), ea(2));
    return ea;
  }

  //! Returns the roll angle of \a pose 's orientation in radians.
  static Scalar roll(const T& pose)
  {
    return ypr(pose)(2);
  }

  //! Returns the pitch angle of \a pose 's orientation in radians.
  static Scalar pitch(const T& pose)
  {
    return ypr(pose)(1);
  }

  //! Returns the yaw angle of \a pose 's orientation in in radians.
  static Scalar yaw(const T& pose)
  {
    return ypr(pose)(0);
  }

  //! Returns just the translational part of \a pose.
  static Position translation(const T& pose)
  {
    return pose.translation();
  }

  //! Returns an identity transform.
  static T identity()
  {
    return T::Identity();
  }

  /*! Set the position part of \a pose to given scalar values \a x, \a
   *  y and \a z.
   */
  static void fromPosition(T& pose, const Scalar x, const Scalar y, const Scalar z)
  {
    const Eigen::Matrix<Scalar, 3, 1> position(x, y, z);
    const Eigen::Matrix<Scalar, 3, 3> orientation = pose.rotation();
    const Eigen::Matrix<Scalar, 3, 1> scale(1., 1., 1.);
    pose.fromPositionOrientationScale(position, orientation, scale);
  }

  /*! Set the orientation part of \a pose to given scalar values \a
   *  roll, \a pitch and \a yaw.
   */
  static void fromOrientationRPY(T& pose, const Scalar roll, const Scalar pitch, const Scalar yaw)
  {
    const Eigen::Matrix<Scalar, 3, 1> position = pose.translation();
    const Eigen::Quaternion<Scalar> orientation =
      Eigen::AngleAxis<Scalar>(yaw,   Eigen::Matrix<Scalar, 3, 1>::UnitZ()) *
      Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()) *
      Eigen::AngleAxis<Scalar>(roll,  Eigen::Matrix<Scalar, 3, 1>::UnitX());
    const Eigen::Matrix<Scalar, 3, 1> scale(1., 1., 1.);
    pose.fromPositionOrientationScale(position, orientation, scale);
  }

  /*! Set the position part of \a pose to given scalar values \a x, \a
   *  y and \a z and the orientation part to scalar values \a roll, \a
   *  pitch and \a yaw.
   */
  static void fromPositionAndOrientationRPY(T& pose,
                                            const Scalar x, const Scalar y, const Scalar z,
                                            const Scalar roll, const Scalar pitch, const Scalar yaw)
  {
    const Eigen::Matrix<Scalar, 3, 1> position(x, y, z);
    const Eigen::Quaternion<Scalar> orientation =
      Eigen::AngleAxis<Scalar>(yaw,   Eigen::Matrix<Scalar, 3, 1>::UnitZ())
      * Eigen::AngleAxis<Scalar>(pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY())
      * Eigen::AngleAxis<Scalar>(roll,  Eigen::Matrix<Scalar, 3, 1>::UnitX());
    const Eigen::Matrix<Scalar, 3, 1> scale(1., 1., 1.);
    pose.fromPositionOrientationScale(position, orientation, scale);
  }

  /*! Constructs a line given by the position and rotation of \a pose,
   *  applied to the given \a direction vector.  By default it returns
   *  a line representing the X axis of the pose.
   *
   *  \param pose The pose
   *  \param line The resulting line
   *  \param direction The direction vector in the pose's coordinate
   *         system. Defaults to UnitX.
   */
  template <int t_options>
  static void toParametrizedLine(const T& pose,
                                 Eigen::ParametrizedLine<Scalar, SPATIAL_DIMENSIONS, t_options>& line,
                                 Position direction = Position::UnitX())
  {
    line = Eigen::ParametrizedLine<Scalar, SPATIAL_DIMENSIONS, t_options>(pose.translation(), pose.rotation() * direction);
  }

  /*! Calculates the \a angle between the \a direction in the \a pose
   *  coordinate system and the X axis of the reference system
   *  projected to the XY plane.  By default it calculates the angle
   *  between the pose's X axis and the reference X axis.
   *
   *  \param pose The pose.
   *  \param angle The resulting angle.
   *  \param direction The direction vector in the pose's coordinate
   *         system.
   */
  static void getAngleXY(const T& pose, double& angle,
                         Position direction = Position::UnitX())
  {
    assert(int(SPATIAL_DIMENSIONS) >= 2 && "this function is only available for dimensions >= 2");
    Position vector = pose.rotation() * direction;
    angle = std::atan2(vector.y(), vector.x());
  }

  /*! Calculates the \a angle between the \a direction in the \a pose
   *  coordinate system and the other_direction in the \a other_pose
   *  coordinate system, all projected to the XY plane.  By default it
   *  calculates the \a angle between the pose's X axis and the
   *  other_pose's X axis.
   *
   *  \todo Maybe this could also be achieved by combining the two transformations and calculating the angle afterwards.
   *
   *  \param pose The pose.
   *  \param other_pose The second pose.
   *  \param direction The direction vector in the pose's coordinate
   *         system.
   *  \param other_direction The direction vector in the second pose's
   *         coordinate system.
   */
  static void getAngleXYDiff(const T &pose, const T &other_pose, double& angle,
                             Position direction = Position::UnitX(),
                             Position other_direction = Position::UnitX())
  {
    double first_angle;
    PoseTraits<T>::getAngleXY(pose, first_angle, direction);
    double second_angle;  // in [- M_PI, M_PI ]
    PoseTraits<T>::getAngleXY(other_pose, second_angle, other_direction);
    angle = normalizeAngleSigned(first_angle - second_angle);
  }

  /*! Get the intersection point of the lines represented by the
   *  direction vectors in the two poses' coordinate system.  By
   *  default this represents the X direction of the pose.
   *
   *  \param pose1 The first pose.
   *  \param pose2 The second pose.
   *  \param[out] intersection_point The resulting intersection point.
   */
  static void getIntersectionPoint(const T& pose1, const T& pose2,
                                   Position& intersection_point,
                                   Position direction1 = Position::UnitX(),
                                   Position direction2 = Position::UnitX())
  {
    Eigen::ParametrizedLine<Scalar, SPATIAL_DIMENSIONS> line1;
    PoseTraits<T>::toParametrizedLine(pose1, line1, direction1);
    Eigen::ParametrizedLine<Scalar, SPATIAL_DIMENSIONS> line2;
    PoseTraits<T>::toParametrizedLine(pose2, line2, direction2);

    intersection_point = line1.intersectionPoint(Eigen::Hyperplane<Scalar, SPATIAL_DIMENSIONS>(line2));
  }

  /*! Helper function to build a pose from two unit vectors
   *  representing the Z and Y axes.
   *  \note similarly defined in PCL transform.hpp
   */
  static void fromUnitVectorsZY(T& pose, const Position& z_axis, const Position& y_direction)
  {
    Position tmp0 = (z_axis.cross(y_direction)).normalized();
    Position tmp1 = (z_axis.cross(tmp0)).normalized();
    Position tmp2 = z_axis.normalized();

    pose(0,0)=tmp0[0]; pose(0,1)=tmp0[1]; pose(0,2)=tmp0[2]; pose(0,3)=0.0f;
    pose(1,0)=tmp1[0]; pose(1,1)=tmp1[1]; pose(1,2)=tmp1[2]; pose(1,3)=0.0f;
    pose(2,0)=tmp2[0]; pose(2,1)=tmp2[1]; pose(2,2)=tmp2[2]; pose(2,3)=0.0f;
    pose(3,0)=0.0f;    pose(3,1)=0.0f;    pose(3,2)=0.0f;    pose(3,3)=1.0f;
  }

  /*! Helper function to build a pose from two unit vectors
   *  representing the X and Y axes.
   *  \note similarly defined in PCL transform.hpp
   */
  static void fromUnitVectorsXY(T& pose, const Position& x_axis, const Position& y_direction)
  {
    Position tmp2 = (x_axis.cross(y_direction)).normalized();
    Position tmp1 = (tmp2.cross(x_axis)).normalized();
    Position tmp0 = x_axis.normalized();

    pose(0,0)=tmp0[0]; pose(0,1)=tmp0[1]; pose(0,2)=tmp0[2]; pose(0,3)=0.0f;
    pose(1,0)=tmp1[0]; pose(1,1)=tmp1[1]; pose(1,2)=tmp1[2]; pose(1,3)=0.0f;
    pose(2,0)=tmp2[0]; pose(2,1)=tmp2[1]; pose(2,2)=tmp2[2]; pose(2,3)=0.0f;
    pose(3,0)=0.0f;    pose(3,1)=0.0f;    pose(3,2)=0.0f;    pose(3,3)=1.0f;
  }

  /**
   * @brief fromTwoUnitVectorsAndOrigin Gets the transformation that will
   * translate origin to (0,0,0) and rotate z_axis into (0,0,1) and y_direction
   * into a vector with x=0 (or into (0,1,0) should y_direction be orthogonal
   * to z_axis)
   *  \note similarly defined in PCL transform.hpp
   */
  static void fromTwoUnitVectorsAndOrigin(T& pose,
                                          const Position& y_direction,
                                          const Position& z_axis,
                                          const Position& origin)
  {
    fromUnitVectorsZY(pose, z_axis, y_direction);
    Position translation = pose*origin;
    pose(0,3)=-translation[0]; pose(1,3)=-translation[1]; pose(2,3)=-translation[2];
  }
};



//! Pose Traits for SE(2) poses.
template <typename T, int t_options>
struct PoseTraits<Eigen::Transform<T, 2, Eigen::Isometry, t_options> >
{
  //! The underlying scalar datatype.
  typedef T Scalar;

  //! Provides the number of dimensions at compile time.
  enum {
    SPATIAL_DIMENSIONS = 2,
    TOTAL_DIMENSIONS   = 3
  };

  //! Position vector type.
  typedef Eigen::Matrix<Scalar, SPATIAL_DIMENSIONS, 1> Position;

  //! Check the poses for equalitiy
  static bool isEqual(const T& pose_a, const T& pose_b)
  {
    return (pose_a.matrix() == pose_b.matrix());
  }

private:
  //! Internal helper typedef representing the actual pose type.
  typedef Eigen::Transform<T, 2, Eigen::Isometry, t_options> PoseType;

public:
  /*! Returns the orientation part of the \a pose in roll-pitch-yaw
   *  convention in radians.
   */
  static Eigen::Matrix<T, 3, 1> rpy(const PoseType& pose)
  {
    Eigen::Rotation2D<T> rot2(0.);
    rot2.fromRotationMatrix(pose.rotation());
    Eigen::Matrix<T, 3, 1> mat;
    mat << 0, 0, rot2.angle();
    return mat;
  }

  //! Returns the roll angle of \a pose 's orientation in radians.
  static T roll(const PoseType& pose)
  {
    return T(0.);
  }

  //! Returns the pitch angle of \a pose 's orientation in radians.
  static T pitch(const PoseType& pose)
  {
    return T(0.);
  }

  //! Returns the yaw angle of \a pose 's orientation in in radians.
  static T yaw(const PoseType& pose)
  {
    Eigen::Rotation2D<T> rot2(0.);
    rot2.fromRotationMatrix(pose.rotation());
    return rot2.angle();
  }

  //! Returns just the translational part of \a pose.
  static Position translation(const PoseType& pose)
  {
    return pose.translation();
  }

  //! Returns an identity transform.
  static PoseType identity()
  {
    return PoseType::Identity();
  }

  /*! Set the position part of \a pose to given scalar values \a x and
   *  \a y.
   */
  static void fromPosition(PoseType& pose,
                           const T x, const T y)
  {
    const Eigen::Matrix<T, 2, 1> position(x, y);
    const Eigen::Matrix<T, 2, 2> orientation = pose.rotation();
    const Eigen::Matrix<T, 2, 1> scale(1., 1.);
    pose.fromPositionOrientationScale(position, orientation, scale);
  }


  /*! Set the orientation part of \a pose to given scalar value \a
   *  yaw.
   */
  static void fromOrientationRPY(PoseType& pose,
                                 const T yaw)
  {
    const Eigen::Matrix<T, 2, 1> position = pose.translation();
    Eigen::Rotation2D<T> orientation(yaw);
    const Eigen::Matrix<T, 2, 1> scale(1., 1.);
    pose.fromPositionOrientationScale(position, orientation, scale);
  }

  /*! Set the position part of \a pose to given scalar values \a x and
   *  \a y and the orientation part to scalar value \a yaw.
   */
  static void fromPositionAndOrientationRPY(PoseType& pose,
                                            const T x, const T y, const T yaw)
  {
    const Eigen::Matrix<T, 2, 1> position(x, y);
    Eigen::Rotation2D<T> orientation(yaw);
    const Eigen::Matrix<T, 2, 1> scale(1., 1.);
    pose.fromPositionOrientationScale(position, orientation, scale);
  }

  /*! Constructs a line given by the position and rotation of \a pose,
   *  applied to the given \a direction vector.  By default it returns
   *  a line representing the X axis of the pose.
   *
   *  \param pose The pose
   *  \param line The resulting line
   *  \param direction The direction vector in the pose's coordinate
   *         system. Defaults to UnitX.
   */
  template <int t_other_options>
  static void toParametrizedLine(const PoseType& pose,
                                 Eigen::ParametrizedLine<Scalar, SPATIAL_DIMENSIONS, t_other_options>& line,
                                 const Position& direction = Position::UnitX())
  {
    line = Eigen::ParametrizedLine<Scalar, SPATIAL_DIMENSIONS, t_other_options>(pose.translation(), pose.rotation() * direction);
  }
};

} // end of ns
} // end of ns

#endif

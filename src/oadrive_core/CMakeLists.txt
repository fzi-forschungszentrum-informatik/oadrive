# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------


ICMAKER_SET("oadrive_core")

ICMAKER_ADD_SOURCES(
  ExtendedPose2d.cpp
  ImportExport.h
  Interpolator.cpp
  Logging.cpp
  Trajectory2d.cpp
  VelocityCurve.cpp
  TrajectoryVelocityInterpolator.cpp
)

ICMAKER_ADD_HEADERS(
  ExtendedPose2d.h
  Interpolator.h
  Logging.h
  MathFunctions.h
  Pose.h
  Trajectory2d.h
  Types.h
  Utility.h
  QuaternionFunctions.h
  VelocityCurve.h
  TrajectoryVelocityInterpolator.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  Eigen3
  icl_core
  icl_core_logging
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_core)

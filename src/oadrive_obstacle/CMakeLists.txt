# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------


ICMAKER_SET("oadrive_obstacle")

ICMAKER_ADD_SOURCES(
  ProcessDepth.cpp
  ProcessSensors.cpp
  ProcessUS.cpp
  obstacleLogging.cpp
  ObjectDetector.cpp
  ObjectTracker.cpp
  KalmanFilter.cpp
)

ICMAKER_ADD_HEADERS(
  ProcessDepth.h
  ProcessSensors.h
  ProcessUS.h
  obstacleLogging.h
  ObjectDetector.h
  ObjectTracker.h
  KalmanFilter.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  OpenCV
  Eigen3
  icl_core
  icl_core_logging
  oadrive_world
  oadrive_util
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_obstacle)

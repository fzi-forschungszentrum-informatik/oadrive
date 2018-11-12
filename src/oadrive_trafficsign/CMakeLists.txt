# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------


ICMAKER_SET("oadrive_trafficsign")

ICMAKER_ADD_SOURCES(
  TrafficSignDetAruco.cpp
  trafficSignLogging.cpp
)

ICMAKER_ADD_HEADERS(
  TrafficSignDetector.h
  TrafficSignDetAruco.h
  trafficSignLogging.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  OpenCV
  aruco
  #Eigen3
  icl_core
  icl_core_logging
  oadrive_core
  #oadrive_control
  #oadrive_util
  oadrive_world
  #oadrive_lanedetection
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_trafficsign)

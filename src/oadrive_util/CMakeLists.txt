# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------


ICMAKER_SET("oadrive_util")

ICMAKER_ADD_SOURCES(
  CoordinateConverter.cpp
  BirdViewConverter.cpp
  InitLogger.cpp
  CameraCalibration.cpp
  utilLogging.cpp
  Timer.cpp
  Config.cpp
  ImageHelper.cpp
  Navigator.cpp
)

ICMAKER_ADD_HEADERS(
  CoordinateConverter.h
  BirdViewConverter.h
  InitLogger.h
  CameraCalibration.h
  utilLogging.h
  Timer.h
  TimerEventListener.h
  TimerTypes.h
  Config.h
  ImageHelper.h
  Navigator.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  OpenCV
  Eigen3
  icl_core
  icl_core_logging
  oadrive_core
  Boost
  Boost_THREAD
)

ICMAKER_DEPENDENCIES(EXPORT OPTIONAL
  redisclient
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_util)

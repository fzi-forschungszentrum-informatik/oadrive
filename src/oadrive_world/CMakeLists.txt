# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------


ICMAKER_SET("oadrive_world")

ICMAKER_ADD_SOURCES(
  EnvObject.cpp
  Environment.cpp
  Patch.cpp
  TrajectoryFactory.cpp
  TrajectoryDatabase.cpp
  Obstacle.cpp
  worldLogging.cpp
  EventRegion.cpp
  TrafficSign.cpp
  EnvironmentPainter.cpp
  Road.cpp
  StateMachine.cpp
)

ICMAKER_ADD_HEADERS(
  aadc_roadSign_enums.h
  EnvObject.h
  Environment.h
  Patch.h
  TrajectoryFactory.h
  TrajectoryDatabase.h
  WorldEventListener.h
  Obstacle.h
  worldLogging.h
  EventRegion.h
  TrafficSign.h
  EnvironmentPainter.h
  Road.h
  StateMachine.h
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
  oadrive_control
  oadrive_util
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_world)

####################################################

ICMAKER_SET("StateMachine")

ICMAKER_ADD_SOURCES(
    StateMachine.cpp
)

ICMAKER_ADD_HEADERS(
    StateMachine.h
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
        oadrive_control
        oadrive_util
        )

ICMAKER_BUILD_PROGRAM()

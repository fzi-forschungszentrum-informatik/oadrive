# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------


########################################################

ICMAKER_SET("test_oadrive_core_trajectory")

ICMAKER_ADD_SOURCES(
  test_oadrive_core_trajectory.cpp
)

ICMAKER_ADD_HEADERS(
)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(
  oadrive_core
)

ICMAKER_BUILD_PROGRAM()

#########################################################

ICMAKER_SET("test_oadrive_haarfilter")

ICMAKER_ADD_SOURCES(
  test_oadrive_haarfilter.cpp
)

ICMAKER_ADD_HEADERS(
)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(
  OpenCV
  oadrive_util
  oadrive_core
  oadrive_lanedetection
)

ICMAKER_BUILD_PROGRAM()

#########################################################

ICMAKER_SET("test_oadrive_drivermodule")

ICMAKER_ADD_SOURCES(
  test_oadrive_drivermodule.cpp
)

ICMAKER_ADD_HEADERS(
)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(
  OpenCV
  oadrive_core
  oadrive_control
  oadrive_world
)

ICMAKER_BUILD_PROGRAM()

#########################################################

ICMAKER_SET("test_oadrive_birdViewCal")

ICMAKER_ADD_SOURCES(
  test_oadrive_birdViewCal.cpp
)

ICMAKER_ADD_HEADERS(
)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(
  OpenCV
  oadrive_util
)

ICMAKER_BUILD_PROGRAM()

#########################################################

#ICMAKER_SET("test_oadrive_trajectoryGeneration.cpp")

#ICMAKER_ADD_SOURCES(
#        test_oadrive_trajectoryGeneration.cpp
#)

#ICMAKER_ADD_HEADERS(
#)

#ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

#ICMAKER_DEPENDENCIES(
#        oadrive_core
#        oadrive_control
#        oadrive_world
#)

#ICMAKER_BUILD_PROGRAM()

#########################################################

#ICMAKER_SET("test_oadrive_lanedetection.cpp")

#ICMAKER_ADD_SOURCES(
#        test_oadrive_lanedetection.cpp
#)

#ICMAKER_ADD_HEADERS(
#)

#ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

#ICMAKER_DEPENDENCIES(
#        oadrive_core
#        oadrive_lanedetection
#        oadrive_world
#)

#ICMAKER_BUILD_PROGRAM()
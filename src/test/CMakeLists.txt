# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

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

ICMAKER_SET("test_oadrive_interface")

ICMAKER_ADD_SOURCES(
        test_oadrive_interface.cpp
)

ICMAKER_ADD_HEADERS(
)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(
        oadrive_core
        oadrive_util
        oadrive_interface
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

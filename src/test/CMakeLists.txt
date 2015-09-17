# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

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


ICMAKER_SET("test_oadrive_core_velocity_interpolator")

ICMAKER_ADD_SOURCES(
  test_oadrive_core_velocity_interpolator.cpp
)

ICMAKER_ADD_HEADERS(
)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(
  oadrive_core
)

ICMAKER_BUILD_PROGRAM()


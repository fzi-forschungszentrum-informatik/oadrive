# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# To be used by other modules
SET(OADRIVE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}/src" CACHE INTERNAL "")
SET(OADRIVE_IDL_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE INTERNAL "")
SET(OADRIVE_IDE_FOLDER "oadrive")

# TODO: place a generic find script into icmaker.
#FIND_PACKAGE(aruco)
FIND_PACKAGE(aruco
  PATHS "/usr/local/lib/cmake"
  "/opt/aruco/1.3.0/lib"
  CONFIGS
  Findaruco.cmake)

FIND_PACKAGE(redisclient PATHS ".")

ICMAKER_REGISTER_PACKAGE(oadrive)

ADD_SUBDIRECTORY(src/oadrive_core)
ADD_SUBDIRECTORY(src/oadrive_control)
ADD_SUBDIRECTORY(src/oadrive_util)
#ADD_SUBDIRECTORY(src/oadrive_trajectory)
ADD_SUBDIRECTORY(src/oadrive_world)
ADD_SUBDIRECTORY(src/oadrive_lanedetection)
ADD_SUBDIRECTORY(src/oadrive_missioncontrol)
ADD_SUBDIRECTORY(src/oadrive_obstacle)
ADD_SUBDIRECTORY(src/oadrive_trafficsign)
ADD_SUBDIRECTORY(src/oadrive_interface)

ICMAKER_CONFIGURE_PACKAGE()

IF(BUILD_TESTS)
#  ADD_SUBDIRECTORY(src/ts)     # unit tests
  ADD_SUBDIRECTORY(src/test)   # standard tests
ENDIF(BUILD_TESTS)

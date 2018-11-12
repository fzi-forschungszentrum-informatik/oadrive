# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
set (CMAKE_CXX_STANDARD 11)

# Make sure oadrive shared libraries can be easily found on macOS systems.
set(CMAKE_MACOSX_RPATH 1)

# Include icmaker if not already available.
#
# When building in an ic_workspace based on icmaker, icmaker is usually globally
# available.
#
# When using catkin (for example in ROS workspaces), icmaker is not globally
# available and thus has to be found and included in every package. Also catkin
# packages usually define a project per package.
#
#
IF(NOT DEFINED ICMAKER_DIRECTORY)
  find_package(icmaker REQUIRED)
  INCLUDE(UseFullIcMaker REQUIRED)
  project(icl_core)
ENDIF()


# To be used by other modules
SET(OADRIVE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}/src" CACHE INTERNAL "")
SET(OADRIVE_IDL_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE INTERNAL "")
SET(OADRIVE_IDE_FOLDER "oadrive")

# find aruco and redisclient
FIND_PACKAGE(aruco)
FIND_PACKAGE(redisclient)

ICMAKER_REGISTER_PACKAGE(oadrive)

ADD_SUBDIRECTORY(src/oadrive_core)
ADD_SUBDIRECTORY(src/oadrive_util)
ADD_SUBDIRECTORY(src/oadrive_control)
ADD_SUBDIRECTORY(src/oadrive_world)
ADD_SUBDIRECTORY(src/oadrive_lanedetection)
ADD_SUBDIRECTORY(src/oadrive_obstacle)

ICMAKER_CONFIGURE_PACKAGE()

IF(BUILD_TESTS)
#  ADD_SUBDIRECTORY(src/ts)     # unit tests
  ADD_SUBDIRECTORY(src/test)   # standard tests
ENDIF(BUILD_TESTS)

# This directory is only included if ROS is available. Prevents us from
# spaming from several ROS-dependent targets.
#ICMAKER_ADD_ROS_SUBDIRECTORY(src/ros)


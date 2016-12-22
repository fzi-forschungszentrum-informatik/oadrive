# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

ICMAKER_SET("oadrive_interface")

ICMAKER_ADD_SOURCES(
	Interface.cpp
        interfaceLogging.cpp
)

ICMAKER_ADD_HEADERS(
	Interface.h
        interfaceLogging.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES( EXPORT
  OpenCV
  oadrive_core
  oadrive_util
  oadrive_lanedetection
  oadrive_world
  oadrive_missioncontrol
  oadrive_obstacle
  oadrive_trafficsign
  Eigen3
  icl_core
  icl_core_logging
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_interface)

# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

ICMAKER_SET("oadrive_obstacle")

ICMAKER_ADD_SOURCES(
  ProcessDepth.cpp
  ProcessSensors.cpp
  ProcessUS.cpp
  obstacleLogging.cpp
)

ICMAKER_ADD_HEADERS(
  ProcessDepth.h
  ProcessSensors.h
  ProcessUS.h
  obstacleLogging.h
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

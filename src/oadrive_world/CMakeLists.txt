# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

ICMAKER_SET("oadrive_world")

ICMAKER_ADD_SOURCES(
  EnvObject.cpp
  Environment.cpp
  Patch.cpp
  PatchStitcher.cpp
  TrajectoryFactory.cpp
  TrajectoryDatabase.cpp
  Obstacle.cpp
  worldLogging.cpp
  EventRegion.cpp
  TrafficSign.cpp
  EnvironmentPainter.cpp
  #PatchStitcherComplex.cpp
)

ICMAKER_ADD_HEADERS(
  EnvObject.h
  Environment.h
  Patch.h
  PatchStitcher.h
  TrajectoryFactory.h
  TrajectoryDatabase.h
  WorldEventListener.h
  Obstacle.h
  worldLogging.h
  EventRegion.h
  TrafficSign.h
  EnvironmentPainter.h
  #PatchStitcherComplex.h
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

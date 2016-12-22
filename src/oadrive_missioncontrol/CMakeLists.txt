# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

ICMAKER_SET("oadrive_missioncontrol")

ICMAKER_ADD_SOURCES(
  MissionControl.cpp
  DebugControl4MC.cpp
  ManeuverList.cpp
  ManeuverListLive.cpp
  mcLogging.cpp
  StateMachine.h
)

ICMAKER_ADD_HEADERS(
  MissionControl.h
  juryEnums.h
  IControl4MC.h
  DebugControl4MC.h
  ManeuverList.h
  IManeuverList.h
  ManeuverListLive.h
  mcLogging.h
  IMC2Man.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  #OpenCV
  #Eigen3
  icl_core
  icl_core_logging
  oadrive_core
  oadrive_control
  oadrive_util
  oadrive_world
  oadrive_lanedetection
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_missioncontrol)

# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

ICMAKER_SET("oadrive_control")

ICMAKER_ADD_SOURCES(
  LateralController.cpp
  DriverModule.cpp
  controlLogging.cpp
)

ICMAKER_ADD_HEADERS(
  LateralController.h
  DriverModule.h
  controlLogging.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  OpenCV
  Eigen3
  icl_core
  icl_core_logging
  oadrive_util
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_control)

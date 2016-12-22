# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

ICMAKER_SET("oadrive_lanedetection")

ICMAKER_ADD_SOURCES(
  HaarFeature.cpp
  HaarFilter.cpp
  # Line.cpp
  IntegralImage.cpp
  StreetPatcher.cpp
  HaarFeatureEvaluation.cpp
  lanedetectionLogging.cpp
)

ICMAKER_ADD_HEADERS(
  HaarFeature.h
  HaarFilter.h
  #Line.h
  IntegralImage.h
  StreetPatcher.h
  StreetTypes.h
  HaarFeatureEvaluation.h
  lanedetectionLogging.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  OpenCV
  oadrive_core
  oadrive_util
  oadrive_world
#  Eigen3
  icl_core
  icl_core_logging
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_lanedetection)

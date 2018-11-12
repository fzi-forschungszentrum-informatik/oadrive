# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# This program is free software licensed under the CDDL
# (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
# You can find a copy of this license in LICENSE in the top
# directory of the source code.
#
# © Copyright 2018 FZI Forschungszentrum Informatik, Karlsruhe, Germany
# -- END LICENSE BLOCK ------------------------------------------------


ICMAKER_SET("oadrive_lanedetection")

ICMAKER_ADD_SOURCES(
        FeatureDetection/HaarFeature.cpp
        FeatureDetection/HaarFilter.cpp
        FeatureDetection/IntegralImage.cpp
        RoadPatching/StreetPatcher.cpp
        FeatureDetection/HaarFeatureEvaluation.cpp
        lanedetectionLogging.cpp
        RoadPatching/IntersectionPatcher.cpp
        RoadPatching/MapPatcher.cpp
        RoadPatching/Map.cpp
        RoadPatching/RoadPatcher.cpp
        RoadPatching/RoadPerception.cpp
        RoadPatching/VotingSpace.cpp
        DebugViewRoadPerception.cpp
        RoadPatching/PatchHypothesis.cpp
        RoadPatching/StreetVotingSpace.cpp
        RoadPatching/IntersectionVotingSpace.cpp
        RoadPatching/FeatureClassification.cpp
        RoadPatching/RoadHypothesis.cpp
        #RoadPatching/ParkingLotVotingSpace.cpp
        # RoadPatching/ParkingLotPatcher.cpp
)

ICMAKER_ADD_HEADERS(
        FeatureDetection/HaarFeature.h
        FeatureDetection/HaarFilter.h
        FeatureDetection/IntegralImage.h
        RoadPatching/StreetPatcher.h
        FeatureDetection/StreetTypes.h
        FeatureDetection/HaarFeatureEvaluation.h
        lanedetectionLogging.h
        RoadPatching/IntersectionPatcher.h
        RoadPatching/MapPatcher.h
        RoadPatching/Map.h
        RoadPatching/RoadPatcher.h
        RoadPatching/RoadPerception.h
        RoadPatching/VotingSpace.h
        DebugViewRoadPerception.h
        RoadPatching/PatchHypothesis.h
        RoadPatching/StreetVotingSpace.h
        RoadPatching/IntersectionVotingSpace.h
        RoadPatching/RoadPerceptionConfig.h
        RoadPatching/FeatureClassification.h
        RoadPatching/RoadHypothesis.h
        # RoadPatching/ParkingLotVotingSpace.h
        # RoadPatching/ParkingLotPatcher.h
)

ICMAKER_LOCAL_CPPDEFINES(-DOADRIVE_EXPORT_SYMBOLS)
ICMAKER_GLOBAL_CPPDEFINES(-D_IC_BUILDER_OADRIVE_)

ICMAKER_INCLUDE_DIRECTORIES(${OADRIVE_INCLUDE_DIRS})

ICMAKER_DEPENDENCIES(EXPORT
  OpenCV
  oadrive_core
  oadrive_util
  oadrive_world
  icl_core
  icl_core_logging
  tinyxml
)

ICMAKER_BUILD_LIBRARY()

ICMAKER_INSTALL_GLOBHEADERS(oadrive_lanedetection)

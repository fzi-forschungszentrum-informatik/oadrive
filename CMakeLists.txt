# this is for emacs file handling -*- mode: cmake; indent-tabs-mode: nil -*-

# To be used by other modules
SET(OADRIVE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" "${CMAKE_CURRENT_BINARY_DIR}/src" CACHE INTERNAL "")
SET(OADRIVE_IDL_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE INTERNAL "")
SET(OADRIVE_IDE_FOLDER "oadrive")

ICMAKER_REGISTER_PACKAGE(oadrive)

ADD_SUBDIRECTORY(src/oadrive_core)
ADD_SUBDIRECTORY(src/oadrive_control)
#ADD_SUBDIRECTORY(src/oadrive_motion)
#ADD_SUBDIRECTORY(src/oadrive_vision)

ICMAKER_CONFIGURE_PACKAGE()

IF(BUILD_TESTS)
#  ADD_SUBDIRECTORY(src/ts)     # unit tests
  ADD_SUBDIRECTORY(src/test)   # standard tests
ENDIF(BUILD_TESTS)

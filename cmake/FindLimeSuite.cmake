# - Try to find the LimeSuite library
# Once done this defines
#
#  LIMESUITE_FOUND - system has liblimesuite
#  LIMESUITE_INCLUDE_DIRS - the liblimesuite include directory
#  LIMESUITE_LIBRARIES - Link these to use liblimesuite
#  Copyright 2016-2017 Jiang Wei <jiangwei@jiangwei.org>
#

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_LIMESUITE QUIET limesuite)

if (LIMESUITE_INCLUDE_DIRS AND LIMESUITE_LIBRARIES)

 set(LIMESUITE_FOUND TRUE)
else (LIMESUITE_INCLUDE_DIRS AND LIMESUITE_LIBRARIES)

  FIND_PATH(LIMESUITE_INCLUDE_DIRS
    NAMES LimeSuite.h
    HINTS $ENV{LIMESUITE_DIR}/include ${PC_LIMESUITE_INCLUDEDIR}
    PATHS /usr/local/include/lime /usr/include/lime /usr/local/include
    /opt/local/include/lime
    ${LIMESUITE_INCLUDE_DIRS}
  )
  
set(limesuite_library_names LimeSuite)

FIND_LIBRARY(
    LIMESUITE_LIBRARIES
    NAMES ${limesuite_library_names}
    HINTS $ENV{LIMESUITE_DIR}/lib
        ${PC_LIMESUITE_LIBDIR}
    PATHS /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

  if(LIMESUITE_INCLUDE_DIRS)
    set(CMAKE_REQUIRED_INCLUDES ${LIMESUITE_INCLUDE_DIRS})
  endif()

  if(LIMESUITE_LIBRARIES)
    set(CMAKE_REQUIRED_LIBRARIES ${LIMESUITE_LIBRARIES})
  endif()

  include(FindPackageHandleStandardArgs)
  FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIMESUITE DEFAULT_MSG LIMESUITE_LIBRARIES LIMESUITE_INCLUDE_DIRS)
  MARK_AS_ADVANCED(LIMESUITE_INCLUDE_DIRS LIMESUITE_LIBRARIES)


endif (LIMESUITE_INCLUDE_DIRS AND LIMESUITE_LIBRARIES)
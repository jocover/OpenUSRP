########################################################################
# Find the library for the USRP Hardware Driver
########################################################################

INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_UHD uhd)

if (UHD_INCLUDE_DIRS AND UHD_LIBRARIES)

 set(UHD_FOUND TRUE)
else (UHD_INCLUDE_DIRS AND UHD_LIBRARIES)

FIND_PATH(
    UHD_INCLUDE_DIRS
    NAMES uhd/config.hpp
    HINTS $ENV{UHD_DIR}/include
        ${PC_UHD_INCLUDEDIR}
    PATHS /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    UHD_LIBRARIES
    NAMES uhd
    HINTS $ENV{UHD_DIR}/lib
        ${PC_UHD_LIBDIR}
    PATHS /usr/local/lib
          /usr/lib
)

  if(UHD_INCLUDE_DIRS)
    set(CMAKE_REQUIRED_INCLUDES ${UHD_INCLUDE_DIRS})
  endif()

  if(UHD_LIBRARIES)
    set(CMAKE_REQUIRED_LIBRARIES ${UHD_LIBRARIES})
  endif()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(UHD DEFAULT_MSG UHD_LIBRARIES UHD_INCLUDE_DIRS)
MARK_AS_ADVANCED(UHD_LIBRARIES UHD_INCLUDE_DIRS)

endif (UHD_INCLUDE_DIRS AND UHD_LIBRARIES)
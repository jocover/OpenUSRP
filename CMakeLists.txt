# Copyright 2016-2017 Jiang Wei <jiangwei@jiangwei.org>
# Copyright 2012-2013,2015 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########################################################################
# This file included, use CMake directory variables
########################################################################

########################################################################
# Conditionally configure the LimeSDR support
########################################################################

#LIBUHD mode

if(ENABLE_LIBUHD)

LIBUHD_REGISTER_COMPONENT("LimeSDR" ENABLE_LIMESDR ON "ENABLE_LIBUHD" OFF OFF)

IF(ENABLE_LIMESDR)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_STANDARD 11)

find_package(LimeSuite REQUIRED)

if (NOT LIMESUITE_FOUND)
  message(WARNING "LimeSuite not found -- required for FackUSRP support")
endif()

message(STATUS "LimeSuite include directories: ${LIMESUITE_INCLUDE_DIRS}")
message(STATUS "LimeSuite libraries: ${LIMESUITE_LIBRARIES}")

message(STATUS "Checking uhd::device::register_device() API...")
message(STATUS "  Reading ${CMAKE_SOURCE_DIR}/include/uhd/device.hpp...")
file(READ ${CMAKE_SOURCE_DIR}/include/uhd/device.hpp device_hpp)
string(FIND "${device_hpp}" "device_filter_t" has_device_filter)
if ("${has_device_filter}" STREQUAL "-1")
    message(STATUS "  has original API")
else()
    add_definitions(-DUHD_HAS_DEVICE_FILTER)
    message(STATUS "  has filter API")
endif()

message(STATUS "Checking uhd::property::set_publisher() API...")
message(STATUS "  Reading ${CMAKE_SOURCE_DIR}/include/uhd/property_tree.hpp...")
file(READ ${CMAKE_SOURCE_DIR}/include/uhd/property_tree.hpp property_tree_hpp)
string(FIND "${property_tree_hpp}" "set_publisher" has_set_publisher)
if ("${has_set_publisher}" STREQUAL "-1")
    message(STATUS "  missing set_publisher() API")
else()
    add_definitions(-Dpublish=set_publisher)
    add_definitions(-Dsubscribe=add_desired_subscriber)
    add_definitions(-Dcoerce=add_coerced_subscriber)
    message(STATUS "  has set_publisher() API")
endif()

    LIBUHD_APPEND_SOURCES(
        ${CMAKE_CURRENT_SOURCE_DIR}/limesdr_impl.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/limesdr_io_impl.cpp
    )

    INCLUDE_DIRECTORIES(${LIMESUITE_INCLUDE_DIRS})
    LIBUHD_APPEND_LIBS(${LIMESUITE_LIBRARIES})
    
ENDIF(ENABLE_LIMESDR)

else()

#UHD MODULE mode

cmake_minimum_required(VERSION 3.1.3)
project(FackUSRP CXX C)
enable_testing()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(CMAKE_CXX_STANDARD 11)

#select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

find_package(UHD REQUIRED)

if (NOT UHD_FOUND)
    message(WARNING "UHD not found -- required for FackUSRP support")
endif()

if (NOT UHD_ROOT)
    get_filename_component(UHD_ROOT "${UHD_INCLUDE_DIRS}/.." ABSOLUTE)
endif()
message(STATUS "UHD root directory: ${UHD_ROOT}")
message(STATUS "UHD include directories: ${UHD_INCLUDE_DIRS}")
message(STATUS "UHD libraries: ${UHD_LIBRARIES}")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${UHD_INCLUDE_DIRS})

message(STATUS "Checking uhd::device::register_device() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/device.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/device.hpp device_hpp)
string(FIND "${device_hpp}" "device_filter_t" has_device_filter)
if ("${has_device_filter}" STREQUAL "-1")
    message(STATUS "  has original API")
else()
    add_definitions(-DUHD_HAS_DEVICE_FILTER)
    message(STATUS "  has filter API")
endif()

message(STATUS "Checking uhd::property::set_publisher() API...")
message(STATUS "  Reading ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp...")
file(READ ${UHD_INCLUDE_DIRS}/uhd/property_tree.hpp property_tree_hpp)
string(FIND "${property_tree_hpp}" "set_publisher" has_set_publisher)
if ("${has_set_publisher}" STREQUAL "-1")
    message(STATUS "  missing set_publisher() API")
else()
    add_definitions(-Dpublish=set_publisher)
    add_definitions(-Dsubscribe=add_desired_subscriber)
    add_definitions(-Dcoerce=add_coerced_subscriber)
    message(STATUS "  has set_publisher() API")
endif()

########################################################################
# Setup LimeSuite
########################################################################

find_package(LimeSuite REQUIRED)

if (NOT LIMESUITE_FOUND)
  message(WARNING "LimeSuite not found -- required for FackUSRP support")
endif()

INCLUDE_DIRECTORIES(${LIMESUITE_INCLUDE_DIRS})

message(STATUS "LimeSuite include directories: ${LIMESUITE_INCLUDE_DIRS}")
message(STATUS "LimeSuite libraries: ${LIMESUITE_LIBRARIES}")

########################################################################
# Setup boost
########################################################################
MESSAGE(STATUS "Configuring Boost C++ Libraries...")

# Although not required on my system, some users have linking issues without
SET(BOOST_REQUIRED_COMPONENTS
    thread
    system
)

if(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")
    list(APPEND BOOST_LIBRARYDIR "/usr/lib64") #fedora 64-bit fix
endif(UNIX AND NOT BOOST_ROOT AND EXISTS "/usr/lib64")

set(Boost_ADDITIONAL_VERSIONS
    "1.35.0" "1.35" "1.36.0" "1.36" "1.37.0" "1.37" "1.38.0" "1.38" "1.39.0" "1.39"
    "1.40.0" "1.40" "1.41.0" "1.41" "1.42.0" "1.42" "1.43.0" "1.43" "1.44.0" "1.44"
    "1.45.0" "1.45" "1.46.0" "1.46" "1.47.0" "1.47" "1.48.0" "1.48" "1.49.0" "1.49"
    "1.50.0" "1.50" "1.51.0" "1.51" "1.52.0" "1.52" "1.53.0" "1.53" "1.54.0" "1.54"
    "1.55.0" "1.55" "1.56.0" "1.56" "1.57.0" "1.57" "1.58.0" "1.58" "1.59.0" "1.59"
    "1.60.0" "1.60" "1.61.0" "1.61" "1.62.0" "1.62" "1.63.0" "1.63" "1.64.0" "1.64"
    "1.65.0" "1.65" "1.66.0" "1.66" "1.67.0" "1.67" "1.68.0" "1.68" "1.69.0" "1.69"
)

find_package(Boost COMPONENTS ${BOOST_REQUIRED_COMPONENTS})

if(NOT Boost_FOUND)
    message(WARNING "Boost not found -- required for FackUSRP support")
    return()
endif()

ADD_DEFINITIONS(-DBOOST_ALL_DYN_LINK)

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

message(STATUS "Boost include directories: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost library directories: ${Boost_LIBRARY_DIRS}")
message(STATUS "Boost libraries: ${Boost_LIBRARIES}")


########################################################################
# Build a UHD module to support FackUSRP
########################################################################
# Targets
add_library(FackUSPR SHARED limesdr_impl.cpp limesdr_io_impl.cpp)
target_link_libraries(FackUSPR
    ${UHD_LIBRARIES}
    ${Boost_LIBRARIES}
    ${LIMESUITE_LIBRARIES})
install(TARGETS FackUSPR
    DESTINATION ${UHD_ROOT}/lib${LIB_SUFFIX}/uhd/modules
)


ENDIF(ENABLE_LIBUHD)
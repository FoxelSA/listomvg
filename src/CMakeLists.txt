 #
 #  listOMVG
 #
 # Copyright (c) 2013-2014 FOXEL SA - http://foxel.ch
 # Please read <http://foxel.ch/license> for more information.
 #
 #
 # Author(s):
 #
 #      Stéphane Flotron <s.flotron@foxel.ch>
 #
 #
 # This file is part of the FOXEL project <http://foxel.ch>.
 #
 # This program is free software: you can redistribute it and/or modify
 # it under the terms of the GNU Affero General Public License as published by
 # the Free Software Foundation, either version 3 of the License, or
 # (at your option) any later version.
 #
 # This program is distributed in the hope that it will be useful,
 # but WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 # GNU Affero General Public License for more details.
 #
 # You should have received a copy of the GNU Affero General Public License
 # along with this program.  If not, see <http://www.gnu.org/licenses/>.
 #
 #
 # Additional Terms:
 #
 #      You are required to preserve legal notices and author attributions in
 #      that material or in the Appropriate Legal Notices displayed by works
 #      containing it.
 #
 #      You are required to attribute the work as explained in the "Usage and
 #      Attribution" section of <http://foxel.ch/license>.
 #/


cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(listOMVG C CXX)

# ==============================================================================
# ListOMVG build options
# ==============================================================================
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_BUILD_TYPE Debug)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -pedantic")
    set(cxx_flags "-std=c++11")
endif()

if (NOT CONFIGURED_ONCE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${warnings} ${cxx_flags}"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${warnings}"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
endif()

set(LISTOMVG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# ==============================================================================
# OpenMP detection
# ==============================================================================
FIND_PACKAGE(OpenMP)
IF (OPENMP_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  IF (NOT MSVC)
    LIST(APPEND listOMVG_LIBRARY_DEPENDENCIES gomp)
  ENDIF (NOT MSVC)
ENDIF (OPENMP_FOUND)


# ==============================================================================
# ListOMVG check for dependencies
# ==============================================================================

# ------------------------------------------------------------------------------
# pkgconfig / opencv
# ------------------------------------------------------------------------------
find_package(PkgConfig)
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------
# libtiff
# ------------------------------------------------------------------------------
pkg_check_modules(PC_LIBTIFF QUIET libtiff)
set(LIBTIFF_DEFINITIONS ${PC_LIBTIFF_CFLAGS_OTHER})

find_path(LIBTIFF_INCLUDE_DIR tiff.h
  HINTS ${PC_LIBTIFF_INCLUDEDIR} ${PC_LIBTIFF_INCLUDE_DIRS}
  PATH_SUFFIXES libtiff )
find_library(LIBTIFF_LIBRARY NAMES tiff libtiff
  HINTS ${PC_LIBTIFF_LIBDIR} ${PC_LIBTIFF_LIBRARY_DIRS} )

# ------------------------------------------------------------------------------
# libxml2
# ------------------------------------------------------------------------------
pkg_check_modules(PC_LIBXML QUIET libxml-2.0)
set(LIBXML2_DEFINITIONS ${PC_LIBXML_CFLAGS_OTHER})

find_path(LIBXML2_INCLUDE_DIR libxml/xpath.h
          HINTS ${PC_LIBXML_INCLUDEDIR} ${PC_LIBXML_INCLUDE_DIRS}
          PATH_SUFFIXES libxml2 )

find_library(LIBXML2_LIBRARY NAMES xml2 libxml2
             HINTS ${PC_LIBXML_LIBDIR} ${PC_LIBXML_LIBRARY_DIRS} )

# ------------------------------------------------------------------------------
# libboost
# ------------------------------------------------------------------------------
pkg_check_modules(PC_LIBBOOST QUIET libboost-filesystem)
set(LIBBOOST_DEFINITIONS ${PC_LIBBOOST_CFLAGS_OTHER})

find_path(LIBBOOST_INCLUDE_DIR boost/filesystem.hpp
  HINTS ${PC_LIBBOOST_INCLUDEDIR} ${PC_LIBBOOST_INCLUDE_DIRS}
          PATH_SUFFIXES boost )

find_library(LIBBOOST_FILESYSTEM_LIBRARY NAMES boost_filesystem libboost_filesystem
   HINTS ${PC_LIBBOOST_LIBDIR} ${PC_LIBBOOST_LIBRARY_DIRS} )
find_library(LIBBOOST_SYSTEM_LIBRARY NAMES boost_system libboost_system
   HINTS ${PC_LIBBOOST_LIBDIR} ${PC_LIBBOOST_LIBRARY_DIRS} )
find_library(LIBBOOST_REGEX_LIBRARY NAMES boost_regex libboost_regex
   HINTS ${PC_LIBBOOST_LIBDIR} ${PC_LIBBOOST_LIBRARY_DIRS} )

# ------------------------------------------------------------------------------
# libelphelphg
# ------------------------------------------------------------------------------
add_custom_target(
  elphelphg
  WORKING_DIRECTORY ${LISTOMVG_SOURCE_DIR}/third_party/elphelphg
  COMMAND cmake ./src/
  COMMAND make
)

set(LIBELPHELPHG_INCLUDE_DIR ${LISTOMVG_SOURCE_DIR}/third_party/elphelphg/src)
set(LIBELPHELPHG_LIBS ${LISTOMVG_SOURCE_DIR}/third_party/elphelphg/elphelphg/libelphelphg.so)

# ------------------------------------------------------------------------------
# stlplus
# ------------------------------------------------------------------------------
add_custom_target(
  stlplus
  WORKING_DIRECTORY ${LISTOMVG_SOURCE_DIR}/third_party/stlplus3/
  COMMAND cmake .
  COMMAND make
)

set(LIBSTL_LIBS ${LISTOMVG_SOURCE_DIR}/third_party/stlplus3/libstlplus.a)

# ==============================================================================
# Third-party libraries:
# ==============================================================================
#ADD_SUBDIRECTORY(third_party)

# ==============================================================================
# Include directories
# ==============================================================================
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIBELPHELPHG_INCLUDE_DIR}
  ${LIBXML2_INCLUDE_DIR}
  ${LIBBOOST_INCLUDE_DIR}
  ${LIBTIFF_INCLUDE_DIR}
  ${LISTOMVG_SOURCE_DIR}/third_party/cmdLine
  ${LISTOMVG_SOURCE_DIR}/third_party/stlAddition
  ${LISTOMVG_SOURCE_DIR}/third_party/stlplus3/filesystemSimplified/
)

# ==============================================================================
# Library list
# ==============================================================================
set(LISTOMVG_LIBRARY_LIST
  ${LIBELPHELPHG_LIBS}
  ${OpenCV_LIBS}
  ${LIBXML2_LIBRARY}
  ${LIBBOOST_SYSTEM_LIBRARY}
  ${LIBBOOST_FILESYSTEM_LIBRARY}
  ${LIBBOOST_REGEX_LIBRARY}
  ${LIBTIFF_LIBRARY}
  ${LIBSTL_LIBS}
  ${listOMVG_LIBRARY_DEPENDENCIES}
)

# ==============================================================================
# Build Executable
# ==============================================================================
add_executable(listOMVG list.cpp)
target_link_libraries(listOMVG
  ${LISTOMVG_LIBRARY_LIST}
)

add_dependencies(listOMVG elphelphg stlplus)

set(CONFIGURED_ONCE TRUE CACHE INTERNAL
    "A flag showing that CMake has configured at least once.")

 #
 #  listomvg
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
 # along with this program.  if not, see <http://www.gnu.org/licenses/>.
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

project(listomvg C CXX)

# ==============================================================================
#  C / C++ flags
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

set(LISTOMVG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# ==============================================================================
# add a target to generate API documentation with Doxygen
# ===============================================================================
find_package(Doxygen)
if(DOXYGEN_FOUND)
  add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
  )
endif(DOXYGEN_FOUND)

# ==============================================================================
# OpenMP detection
# ==============================================================================
find_package(OpenMP)
if (OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  if (NOT MSVC)
    LIST(APPEND LISTOMVG_LIBRARY_DEPENDENCIES gomp)
  endif (NOT MSVC)
endif (OPENMP_FOUND)


# ==============================================================================
# listomvg check for dependencies
# ==============================================================================

# ------------------------------------------------------------------------------
# pkgconfig / opencv
# ------------------------------------------------------------------------------
find_package(PkgConfig)
find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------
# libfastcal
# ------------------------------------------------------------------------------
add_custom_target(
  fastcal
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/libfastcal
  COMMAND make
)

set(LIBFASTCAL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/libfastcal/src)
set(LIBFASTCAL_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/lib/libfastcal/bin/libfastcal.a)

# ------------------------------------------------------------------------------
# stlplus
# ------------------------------------------------------------------------------
add_subdirectory(lib)
set(LIBSTL_LIBS ${CMAKE_BINARY_DIR}/lib/stlplus3/libstlplus.a)

# ==============================================================================
# Include directories
# ==============================================================================
include_directories(
  ${LISTOMVG_SOURCE_DIR}
  ${LIBFASTCAL_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/cmdLine
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stlAddition
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/stlplus3/filesystemSimplified/
)

# ==============================================================================
# Library list
# ==============================================================================
set(LISTOMVG_LIBRARY_LIST
  ${LIBFASTCAL_LIBS}
  ${OpenCV_LIBS}
  ${LIBSTL_LIBS}
  ${LISTOMVG_LIBRARY_DEPENDENCIES}
)

# ==============================================================================
# Build Executable
# ==============================================================================
add_subdirectory(src)

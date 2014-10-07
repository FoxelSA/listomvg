cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(listOMVG C CXX)

set(CMAKE_CXX_COMPILER g++)

set(CMAKE_BUILD_TYPE Debug)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
    "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    set(warnings "-Wall -Wextra -Werror")
    set(cxx_flags "-std=c++11")
endif()

if (NOT CONFIGURED_ONCE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${warnings} ${cxx_flags}"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
    set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${warnings}"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
endif()

set(LISTOMVG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

find_package(PkgConfig)

# libtiff
pkg_check_modules(PC_LIBTIFF QUIET libtiff)
set(LIBTIFF_DEFINITIONS ${PC_LIBTIFF_CFLAGS_OTHER})

find_path(LIBTIFF_INCLUDE_DIR tiff.h
  HINTS ${PC_LIBTIFF_INCLUDEDIR} ${PC_LIBTIFF_INCLUDE_DIRS}
  PATH_SUFFIXES libtiff )
find_library(LIBTIFF_LIBRARY NAMES tiff libtiff
  HINTS ${PC_LIBTIFF_LIBDIR} ${PC_LIBTIFF_LIBRARY_DIRS} )

# libxml2
pkg_check_modules(PC_LIBXML QUIET libxml-2.0)
set(LIBXML2_DEFINITIONS ${PC_LIBXML_CFLAGS_OTHER})

find_path(LIBXML2_INCLUDE_DIR libxml/xpath.h
          HINTS ${PC_LIBXML_INCLUDEDIR} ${PC_LIBXML_INCLUDE_DIRS}
          PATH_SUFFIXES libxml2 )

find_library(LIBXML2_LIBRARY NAMES xml2 libxml2
             HINTS ${PC_LIBXML_LIBDIR} ${PC_LIBXML_LIBRARY_DIRS} )

# libboost
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

# libelphelphg
add_custom_target(
  elphelphg
  WORKING_DIRECTORY ${LISTOMVG_SOURCE_DIR}/../third_party/elphelphg
  COMMAND cmake ./src/
  COMMAND make
)

set(LIBELPHELPHG_INCLUDE_DIR ${LISTOMVG_SOURCE_DIR}/../third_party/elphelphg/src)
set(LIBELPHELPHG_LIBS ${LISTOMVG_SOURCE_DIR}/../third_party/elphelphg/elphelphg/libelphelphg.so)

# includes
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIBELPHELPHG_INCLUDE_DIR}
  ${LIBXML2_INCLUDE_DIR}
  ${LIBBOOST_INCLUDE_DIR}
  ${LIBTIFF_INCLUDE_DIR}
)

# libraries
set(LISTOMVG_LIBRARY_LIST
  ${LIBELPHELPHG_LIBS}
  ${OpenCV_LIBS}
  ${LIBXML2_LIBRARY}
  ${LIBBOOST_SYSTEM_LIBRARY}
  ${LIBBOOST_FILESYSTEM_LIBRARY}
  ${LIBBOOST_REGEX_LIBRARY}
  ${LIBTIFF_LIBRARY}
 #  ${CMAKE_THREAD_LIBS_INIT}
)
# build listOMVG
add_executable(listOMVG list.cpp)
target_link_libraries(listOMVG
  ${LISTOMVG_LIBRARY_LIST}
)

add_dependencies(listOMVG elphelphg)

set(CONFIGURED_ONCE TRUE CACHE INTERNAL
    "A flag showing that CMake has configured at least once.")

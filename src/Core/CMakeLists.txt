cmake_minimum_required(VERSION 3.0)

project(MapCreator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

aux_source_directory(
    ${CMAKE_CURRENT_SOURCE_DIR}
    MapCreatorCore_SOURCE_FILES)

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)
include(CheckCXXCompilerFlag)

set(MapCreatorCore_INCLUDE_DIR ../../include/Core)
include_directories(${MapCreatorCore_INCLUDE_DIR})

find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package(
  Boost
  COMPONENTS
  system
  filesystem
  serialization
  iostreams
  REQUIRED)
find_package(log4cxx)
find_package (Eigen3 3.3 REQUIRED)

add_library(
    mapcreator_core
    SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/Image.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Feature.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Matcher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Grabber.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PointCloud.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/Inliers.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/Calibrator.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/KeyFrame.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/Transformation.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Utility.cpp
)

target_link_libraries(
    mapcreator_core
    PUBLIC
    ${LOG4CXX_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

# install (TARGETS MapCreatorCore DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")

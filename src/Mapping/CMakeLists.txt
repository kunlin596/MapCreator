cmake_minimum_required (VERSION 3.0)

project(mapcreatorslam VERSION 0.1.0 LANGUAGES CXX)

aux_source_directory (${CMAKE_CURRENT_SOURCE_DIR} mapcreatorslam_SOURCE_FILES)

set(CMAKE_CXX_FLAGS "-O3 -std=c++14 -Wall")

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)
include(CheckCXXCompilerFlag)

find_package(PkgConfig)
pkg_check_modules(LOG4CXX liblog4cxx)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(glm REQUIRED)
find_package(Boost COMPONENTS system filesystem serialization iostreams REQUIRED)

link_directories(${LOG4CXX_LIBDIR})
set(mapcreatorslam_INCLUDE_DIR ../../include)

include_directories (${mapcreatorslam_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${LOG4CXX_INCLUDEDIR})

add_library(mapcreatorslam_keyframe SHARED ${CMAKE_CURRENT_SOURCE_DIR}/KeyFrame.cpp)
target_link_libraries(mapcreatorslam_keyframe PUBLIC ${OpenCV_LIBS} glm)

add_library(mapcreatorslam_matches SHARED ${CMAKE_CURRENT_SOURCE_DIR}/Matcher.cpp)
target_link_libraries(mapcreatorslam_matches PUBLIC ${OpenCV_LIBS} glm)

add_library(mapcreatorslam_coordconverter SHARED ${CMAKE_CURRENT_SOURCE_DIR}/CoordinateConverter.cpp)
target_link_libraries(mapcreatorslam_coordconverter PUBLIC ${OpenCV_LIBS} glm)

# install (TARGETS MapCreatorCore	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")

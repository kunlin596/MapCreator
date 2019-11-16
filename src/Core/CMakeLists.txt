cmake_minimum_required (VERSION 3.0)

project(MapCreatorCore VERSION 0.1.0 LANGUAGES CXX)

aux_source_directory (${CMAKE_CURRENT_SOURCE_DIR} MapCreatorCore_SOURCE_FILES)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wall")

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckLibraryExists)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)
include(CheckCXXSourceRuns)
include(CheckCXXCompilerFlag)

find_package(PkgConfig)
find_package(OpenCV REQUIRED)
find_package (glm REQUIRED)
find_package (Boost COMPONENTS system filesystem serialization iostreams REQUIRED)
pkg_check_modules(LOG4CXX liblog4cxx)

link_directories(${LOG4CXX_LIBDIR})
set(MapCreatorCore_INCLUDE_DIR ../../include/Core)

include_directories (${MapCreatorCore_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${LOG4CXX_INCLUDEDIR})

add_library(mapcreatorcore_math SHARED ${CMAKE_CURRENT_SOURCE_DIR}/MyMath.cpp)
target_link_libraries(mapcreatorcore_math PUBLIC ${OpenCV_LIBS} glm)

add_library(mapcreatorcore_utility SHARED ${CMAKE_CURRENT_SOURCE_DIR}/Utility.cpp)
target_link_libraries(mapcreatorcore_utility PUBLIC ${OpenCV_LIBS} glm)

add_library(mapcreatorcore_image SHARED ${CMAKE_CURRENT_SOURCE_DIR}/Image.cpp)
target_link_libraries(mapcreatorcore_image PUBLIC ${OpenCV_LIBS} glm)

add_library(mapcreatorcore_pointcloud SHARED ${CMAKE_CURRENT_SOURCE_DIR}/PointCloud.cpp)
target_link_libraries(mapcreatorcore_pointcloud PUBLIC ${OpenCV_LIBS})

add_library(mapcreatorcore_feature SHARED ${CMAKE_CURRENT_SOURCE_DIR}/Feature.cpp)
target_link_libraries(mapcreatorcore_feature PUBLIC ${OpenCV_LIBS} ${Boost_LIBRARIES} ${LOG4CXX_LIBRARIES} glm)

# install (TARGETS MapCreatorCore	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")

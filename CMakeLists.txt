cmake_minimum_required ( VERSION 2.8 )

project (MapCreator)

set (MapCreator_VERSION_MAJOR 0)
set (MapCreator_VERSION_MINOR 1)
set (MapCreator_VERSION_PATCH 0)
set (MapCreator_VERSION 0)

set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
set ( MapCreator_INSTALL_INCLUDE_DIR "${MapCreator_SOURCE_DIR}/include" )
set ( MapCreator_LIB_DIR "${MapCreator_SOURCE_DIR}/lib" )

set ( MapCreator_INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/lib )
set ( CMAKE_MACOSX_RPATH ${CMAKE_INSTALL_PREFIX}/lib )

include_directories ( ${MapCreator_INSTALL_INCLUDE_DIR} )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
# set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

add_subdirectory ( src )


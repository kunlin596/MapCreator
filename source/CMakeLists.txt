cmake_minimum_required ( VERSION 2.8 )

project ( NiS )

set ( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
#set ( CMAKE_INSTALL_PREFIX "../install" )
set ( NiS_INCLUDE_DIR "${NiS_SOURCE_DIR}/include" )
set ( NiS_LIB_DIR "${NiS_SOURCE_DIR}/lib" )

set ( INSTALL_NAME_DIR ${CMAKE_INSTALL_PREFIX}/lib )
set ( CMAKE_MACOSX_RPATH ${CMAKE_INSTALL_PREFIX}/lib )

include_directories ( ${NiS_INCLUDE_DIR} )
add_subdirectory ( include )
add_subdirectory ( lib )
add_subdirectory ( tool )


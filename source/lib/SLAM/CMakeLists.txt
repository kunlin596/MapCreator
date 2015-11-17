cmake_minimum_required ( VERSION 2.8 )

project ( NiSSLAM )

#set ( CMAKE_MACOSX_RPATH 1 )

aux_source_directory ( . SLAM_SOURCE_FILES )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

# For aruco settings
# http://www.uco.es/investiga/grupos/ava/node/26
set ( CMAKE_MODULE_PATH /usr/local/lib/cmake )

find_package ( Qt5Widgets REQUIRED )
find_package ( Qt5OpenGL REQUIRED )
find_package ( OpenCV REQUIRED )
find_package ( Boost COMPONENTS system filesystem serialization REQUIRED )
find_package ( aruco REQUIRED )

qt5_wrap_cpp ( SLAM_MOC_FILES "${NiS_INCLUDE_DIR}/SLAM/Alignment.h" )

include_directories ( ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} )

add_library ( NiSSLAM SHARED
	${SLAM_SOURCE_FILES}
	${SLAM_MOC_FILES} )

target_link_libraries ( NiSSLAM
	NiSCore
	${OpenCV_LIBS}
	${aruco_LIBS}
	${Boost_LIBRARIES}
	Qt5::Widgets
	Qt5::OpenGL )

install ( TARGETS
	NiSSLAM
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )


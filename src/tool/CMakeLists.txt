cmake_minimum_required ( VERSION 2.8 )

#set ( CMAKE_MACOSX_RPATH ${CMAKE_MACOSX_RPATH} ${CMAKE_INSTALL_PREFIX}/lib )
set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

find_package ( Boost COMPONENTS system filesystem REQUIRED )
find_package ( OpenCV REQUIRED )
find_package ( Qt5Widgets REQUIRED )
find_package ( Qt5Concurrent REQUIRED )
find_package ( Qt5OpenGL REQUIRED )

include_directories ( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${NiS_INCLUDE_DIR} )

set ( CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib" )

add_executable ( MapCreator
	MapCreator.cpp )

target_link_libraries ( MapCreator
	MapCreatorCore
	MapCreatorHandler
	MapCreatorSlam
	MapCreatorEngine
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	Qt5::Widgets
	Qt5::Concurrent
	Qt5::OpenGL )

add_executable ( Viewer Viewer.cpp )
add_executable ( FrameViewer FrameViewer.cpp )

target_link_libraries ( FrameViewer
	MapCreatorCore
	MapCreatorSlam
	MapCreatorEngine
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	Qt5::Widgets
	Qt5::Concurrent
	Qt5::OpenGL )


install ( TARGETS
	MapCreator
	FrameViewer
	DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" )
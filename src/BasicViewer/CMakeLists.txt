cmake_minimum_required ( VERSION 2.8 )

#set ( CMAKE_MACOSX_RPATH 1 )

aux_source_directory ( . BasicViewer_SOURCE_FILES )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

# For aruco settings
# http://www.uco.es/investiga/grupos/ava/node/26
set ( CMAKE_MODULE_PATH /usr/local/lib/cmake )

find_package ( Boost COMPONENTS system filesystem REQUIRED )
find_package ( OpenCV REQUIRED )
find_package ( Qt5Widgets REQUIRED )
find_package ( Qt5Concurrent REQUIRED )
find_package ( Qt5OpenGL REQUIRED )

qt5_wrap_ui ( BasicViewer_UIC_FILES
	${MapCreator_SOURCE_DIR}/src/BasicViewer/form/BasicViewer.ui
	${MapCreator_SOURCE_DIR}/src/BasicViewer/form/FrameViewer.ui
	${MapCreator_SOURCE_DIR}/src/BasicViewer/form/EvaluationViewer.ui )

qt5_add_resources ( BasicViewer_RCC_FILES
	${MapCreator_SOURCE_DIR}/src/BasicViewer/rsc/Shaders.qrc )

qt5_wrap_cpp ( BasicViewer_MOC_FILES
	${MapCreator_INCLUDE_DIR}/BasicViewer/BasicViewer.h
	${MapCreator_INCLUDE_DIR}/BasicViewer/FrameViewer.h
	${MapCreator_INCLUDE_DIR}/BasicViewer/BaseViewer.h
	${MapCreator_INCLUDE_DIR}/BasicViewer/CloudViewer.h
	${MapCreator_INCLUDE_DIR}/BasicViewer/EvaluationViewer.h )

include_directories ( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )

add_library ( MapCreatorViewer SHARED
	${BasicViewer_SOURCE_FILES}
	${BasicViewer_UIC_FILES}
	${BasicViewer_RCC_FILES}
	${BasicViewer_MOC_FILES} )

target_link_libraries ( MapCreatorViewer
	MapCreatorCore
	MapCreatorSLAM
	MapCreatorHandler
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	${aruco_LIBS}
	Qt5::Widgets
	Qt5::Concurrent
	Qt5::OpenGL )

install ( TARGETS
	MapCreatorViewer
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )

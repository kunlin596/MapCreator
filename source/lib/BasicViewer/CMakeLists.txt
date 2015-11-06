cmake_minimum_required ( VERSION 2.8 )

project ( NiSBasicViewer )

#set ( CMAKE_MACOSX_RPATH 1 )

aux_source_directory ( . BasicViewer_SOURCE_FILES )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

find_package ( Boost COMPONENTS system filesystem REQUIRED )
find_package ( OpenCV REQUIRED )
find_package ( Qt5Widgets REQUIRED )
find_package ( Qt5Concurrent REQUIRED )
find_package ( Qt5OpenGL REQUIRED )

qt5_wrap_ui ( BasicViewer_UIC_FILES
	"${NiS_LIB_DIR}/BasicViewer/form/BasicViewer.ui"
	"${NiS_LIB_DIR}/BasicViewer/form/EvaluationViewer.ui" )

qt5_add_resources ( BasicViewer_RCC_FILES
	"${NiS_LIB_DIR}/BasicViewer/rsc/Shaders.qrc" )

qt5_wrap_cpp ( BasicViewer_MOC_FILES
	"${NiS_INCLUDE_DIR}/BasicViewer/BasicViewer.h"
	"${NiS_INCLUDE_DIR}/BasicViewer/BaseViewer.h"
	"${NiS_INCLUDE_DIR}/BasicViewer/CloudViewer.h"
	"${NiS_INCLUDE_DIR}/BasicViewer/EvaluationViewer.h" )

include_directories ( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} )

add_library ( NiSBasicViewer SHARED
	${BasicViewer_SOURCE_FILES}
	${BasicViewer_UIC_FILES}
	${BasicViewer_RCC_FILES}
	${BasicViewer_MOC_FILES} )

target_link_libraries ( NiSBasicViewer
	NiSCore
	NiSSLAM
	NiSHandler
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	Qt5::Widgets
	Qt5::Concurrent
	Qt5::OpenGL )

install ( TARGETS
	NiSBasicViewer
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )

cmake_minimum_required ( VERSION 2.8 )

project (MapCreator)

aux_source_directory ( . MapCreator_SOURCE_FILES )

#set ( CMAKE_MACOSX_RPATH 1 )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

find_package ( Boost COMPONENTS system filesystem REQUIRED )
find_package ( OpenCV REQUIRED )
find_package ( Qt5Widgets REQUIRED )
find_package ( Qt5Core REQUIRED )
find_package ( Qt5Concurrent REQUIRED )
find_package ( Qt5OpenGL REQUIRED )

qt5_wrap_ui ( MapCreator_UIC_FILES
	${MapCreator_SOURCE_DIR}/form/LogPanelDialog.ui
	${MapCreator_SOURCE_DIR}/form/MarkerViewerDialog.ui
	${MapCreator_SOURCE_DIR}/form/MarkerSelectorDialog.ui
	${MapCreator_SOURCE_DIR}/form/CoordinateConverterDialog.ui
	${MapCreator_SOURCE_DIR}/form/ComputationConfigureDialog.ui
	${MapCreator_SOURCE_DIR}/form/FixedFrameCount_FrameTrackingMethodDialog.ui
	${MapCreator_SOURCE_DIR}/form/OneByOne_FrameTrackingMethodDialog.ui
	${MapCreator_SOURCE_DIR}/form/PcaKeyFrame_FrameTrackingMethodDialog.ui
	${MapCreator_SOURCE_DIR}/form/InliersViewerOptionDialog.ui
	${MapCreator_SOURCE_DIR}/form/SlamComputationDialog.ui
	${MapCreator_SOURCE_DIR}/form/MainWindow.ui )

qt5_wrap_cpp ( MapCreator_MOC_FILES
	${MapCreator_INCLUDE_DIR}/MapCreator/ComputationConfigureDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/CoordinateConverterDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/PcaKeyFrame_FrameTrackingMethodDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/OneByOne_FrameTrackingMethodDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/FixedFrameCount_FrameTrackingMethodDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/LogPanelDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/MarkerViewerDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/MarkerSelectorDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/InliersViewerOptionDialog.h
	${MapCreator_INCLUDE_DIR}/MapCreator/MainWindow.h )

qt5_add_resources ( MapCreator_RCC_FILES
	${MapCreator_SOURCE_DIR}/rsc/Resource.qrc )

include_directories ( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${MapCreator_INCLUDE_DIR} )

# Build library
add_library ( NiSMapCreator SHARED
	${MapCreator_SOURCE_FILES}
	${MapCreator_UIC_FILES}
	${MapCreator_MOC_FILES}
	${MapCreator_RCC_FILES} )

target_link_libraries ( NiSMapCreator
	NiSCore
	NiSBasicViewer
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	Qt5::Widgets
	Qt5::Core
	Qt5::Concurrent )

install ( TARGETS
	NiSMapCreator
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )

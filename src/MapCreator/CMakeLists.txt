cmake_minimum_required ( VERSION 2.8 )

project ( NiSMapCreator )

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
	${NiS_LIB_DIR}/MapCreator/form/LogPanelDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/MarkerViewerDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/MarkerSelectorDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/CoordinateConverterDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/ComputationConfigureDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/FixedFrameCount_FrameTrackingMethodDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/OneByOne_FrameTrackingMethodDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/PcaKeyFrame_FrameTrackingMethodDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/InliersViewerOptionDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/SlamComputationDialog.ui
	${NiS_LIB_DIR}/MapCreator/form/MainWindow.ui )

qt5_wrap_cpp ( MapCreator_MOC_FILES
	${NiS_INCLUDE_DIR}/MapCreator/ComputationConfigureDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/CoordinateConverterDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/PcaKeyFrame_FrameTrackingMethodDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/OneByOne_FrameTrackingMethodDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/FixedFrameCount_FrameTrackingMethodDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/LogPanelDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/MarkerViewerDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/MarkerSelectorDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/InliersViewerOptionDialog.h
	${NiS_INCLUDE_DIR}/MapCreator/MainWindow.h )

qt5_add_resources ( MapCreator_RCC_FILES
	${NiS_LIB_DIR}/MapCreator/rsc/Resource.qrc )

include_directories ( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${NiS_INCLUDE_DIR} )

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

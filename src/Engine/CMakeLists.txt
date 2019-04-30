cmake_minimum_required ( VERSION 2.8 )

aux_source_directory ( . MapCreatorEngine_SOURCE_FILES )

# set ( CMAKE_MACOSX_RPATH 1 )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

find_package ( Boost COMPONENTS system filesystem REQUIRED )
find_package ( Qt5Concurrent REQUIRED )
find_package ( Qt5OpenGL REQUIRED )

qt5_wrap_ui ( MapCreatorEngine_UIC_FILES
	${MapCreator_SOURCE_DIR}/src/Engine/form/LogPanelDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/MarkerViewerDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/MarkerSelectorDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/CoordinateConverterDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/ComputationConfigureDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/FixedFrameCount_FrameTrackingMethodDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/OneByOne_FrameTrackingMethodDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/PcaKeyFrame_FrameTrackingMethodDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/InliersViewerOptionDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/SlamComputationDialog.ui
	${MapCreator_SOURCE_DIR}/src/Engine/form/MainWindow.ui )

qt5_wrap_cpp ( MapCreatorEngine_MOC_FILES
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/ComputationConfigureDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/CoordinateConverterDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/PcaKeyFrame_FrameTrackingMethodDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/OneByOne_FrameTrackingMethodDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/FixedFrameCount_FrameTrackingMethodDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/LogPanelDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/MarkerViewerDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/MarkerSelectorDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/InliersViewerOptionDialog.h
	${MapCreator_INSTALL_INCLUDE_DIR}/Engine/MainWindow.h )

qt5_add_resources ( MapCreatorEngine_RCC_FILES
	${MapCreator_SOURCE_DIR}/src/Engine/rsc/Resource.qrc )

include_directories ( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${MapCreator_INSTALL_INCLUDE_DIR} ${MapCreatorEngine_UIC_FILES} )

# Build library
add_library ( MapCreatorEngine SHARED
	${MapCreatorEngine_SOURCE_FILES}
	${MapCreatorEngine_MOC_FILES}
	${MapCreatorEngine_RCC_FILES} )

target_link_libraries ( MapCreatorEngine
	MapCreatorCore
	MapCreatorViewer
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	Qt5::Widgets
	Qt5::Core
	Qt5::Concurrent )

install ( TARGETS
	MapCreatorEngine
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )

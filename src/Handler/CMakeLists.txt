cmake_minimum_required ( VERSION 2.8 )

#set ( CMAKE_MACOSX_RPATH 1 )

aux_source_directory ( . MapCreatorHandler_SOURCE_FILES )

# settings for using Qt5
set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

find_package ( Boost COMPONENTS system filesystem REQUIRED )
find_package ( Qt5Core REQUIRED )
find_package ( Qt5Concurrent REQUIRED )
find_package ( OpenCV REQUIRED )

include_directories (
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
        ${MapCreator_INCLUDE_DIR} )

add_library ( MapCreatorHandler SHARED
        ${MapCreatorHandler_SOURCE_FILES}
	${HANDLERS_MOC_FILES} )

target_link_libraries ( MapCreatorHandler
        MapCreatorCore
        MapCreatorSlam
        ${OpenCV_LIBS}
	${Boost_LIBRARIES}
	Qt5::Core
	Qt5::Concurrent )

install ( TARGETS
        MapCreatorHandler
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )


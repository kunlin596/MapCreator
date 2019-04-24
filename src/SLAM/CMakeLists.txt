#set ( CMAKE_MACOSX_RPATH 1 )

aux_source_directory ( . MapCreatorSlam_SOURCE_FILES )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

# For aruco settings
# http://www.uco.es/investiga/grupos/ava/node/26
set ( CMAKE_MODULE_PATH /usr/local/lib/cmake )

find_package ( Qt5Widgets REQUIRED )
find_package ( Qt5OpenGL REQUIRED )
find_package ( OpenCV REQUIRED )
find_package ( Boost COMPONENTS system filesystem serialization REQUIRED )

include_directories ( ${Boost_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${MapCreator_INSTALL_INCLUDE_DIR} )

add_library ( MapCreatorSlam SHARED
        ${MapCreatorSlam_SOURCE_FILES}
        ${MapCreatorSlam_MOC_FILES} )

target_link_libraries ( MapCreatorSlam
        MapCreatorCore
	${OpenCV_LIBS}
	${aruco_LIBS}
	${Boost_LIBRARIES}
	Qt5::Widgets
	Qt5::OpenGL )

install ( TARGETS
        MapCreatorSlam
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )


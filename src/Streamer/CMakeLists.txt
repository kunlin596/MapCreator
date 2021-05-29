cmake_minimum_required ( VERSION 2.8 )

aux_source_directory ( . MapCreatorHandler_SOURCE_FILES )

# set ( CMAKE_MACOSX_RPATH 1 )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

find_package ( Boost COMPONENTS system filesystem REQUIRED )
find_package ( Qt5Concurrent REQUIRED )

qt5_wrap_cpp ( MapCreatorHandler_MOC_FILES
    ${MapCreator_INSTALL_INCLUDE_DIR}/Handler/ImageDataHandler.h
)

include_directories (
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${MapCreator_INSTALL_INCLUDE_DIR}
)

add_library ( MapCreatorHandler SHARED
    ${MapCreatorHandler_SOURCE_FILES}
	${MapCreatorHandler_MOC_FILES} )

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


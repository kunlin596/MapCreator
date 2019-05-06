cmake_minimum_required ( VERSION 2.8 )

#set ( CMAKE_MACOSX_RPATH 1 )

aux_source_directory ( . MapCreatorCore_SOURCE_FILES )

# settings for using Qt5
set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

find_package ( glm REQUIRED )
find_package ( Boost COMPONENTS system filesystem serialization iostreams REQUIRED )

include_directories ( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${MapCreator_INSTALL_INCLUDE_DIR} )

add_library ( MapCreatorCore SHARED ${MapCreatorCore_SOURCE_FILES} )

target_link_libraries ( MapCreatorCore
    ${LOG4CXX_LIBRARIES}
	${OpenCV_LIBS}
	${Boost_LIBRARIES}
	Qt5::Core
	Qt5::OpenGL
    glm )

install ( TARGETS
    MapCreatorCore	
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" )

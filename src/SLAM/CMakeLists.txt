#set ( CMAKE_MACOSX_RPATH 1 )

aux_source_directory ( . MapCreatorSlam_SOURCE_FILES )

set ( CMAKE_INCLUDE_CURRENT_DIR ON )
set ( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5" )

find_package ( Boost COMPONENTS system filesystem serialization REQUIRED )
find_package ( Eigen3 REQUIRED NO_MODULE )

include_directories (
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${MapCreator_INSTALL_INCLUDE_DIR}
)

# qt5_wrap_cpp is necessary for running qt5 moc
qt5_wrap_cpp ( MapCreatorSlam_MOC_FILES
    ${MapCreator_INSTALL_INCLUDE_DIR}/SLAM/Alignment.h
)

add_library ( MapCreatorSlam SHARED
    ${MapCreatorSlam_SOURCE_FILES}
    ${MapCreatorSlam_MOC_FILES}
)

target_link_libraries ( MapCreatorSlam
    MapCreatorCore
    ${OpenCV_LIBS}
    # ${aruco_LIBS}
    ${Boost_LIBRARIES}
    Qt5::Widgets
    Qt5::OpenGL
)

install ( TARGETS
    MapCreatorSlam
	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
)


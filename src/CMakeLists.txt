cmake_minimum_required(VERSION 2.8)

# dependencies
find_package(PkgConfig)

pkg_check_modules(LOG4CXX REQUIRED liblog4cxx)
include_directories(${LOG4CXX_INCLUDEDIR})
link_directories(${LOG4CXX_LIBDIR})

find_package ( Boost COMPONENTS system filesystem REQUIRED )
find_package ( OpenCV REQUIRED)
find_package ( Qt5Core REQUIRED )
find_package ( Qt5OpenGL REQUIRED )
find_package ( Qt5Widgets REQUIRED )

add_subdirectory(Core)
add_subdirectory(SLAM)
add_subdirectory(Handler)
add_subdirectory(BasicViewer)
add_subdirectory(Engine)
add_subdirectory(tool)

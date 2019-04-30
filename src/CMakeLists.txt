cmake_minimum_required(VERSION 2.8)

find_package(OpenCV 3.0 REQUIRED)
find_package ( Qt5Core REQUIRED )
find_package ( Qt5OpenGL REQUIRED )
find_package ( Qt5Widgets REQUIRED )

add_subdirectory(Core)
add_subdirectory(SLAM)
add_subdirectory(Handler)
add_subdirectory(BasicViewer)
add_subdirectory(Engine)
add_subdirectory(tool)

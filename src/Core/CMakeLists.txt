# MapCreatorCore — SLAM core algorithms (absorbed the former SLAM/Mapping module).
file(GLOB MAPCREATOR_CORE_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB MAPCREATOR_CORE_HEADERS CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/Core/*.h)

# Deferred: the high-level orchestration layer (Tracker/Algorithm) still relies on
# the pre-refactor XtionCoordinateConverter/AistCoordinateConverter split that was
# consolidated into a single CoordinateConverter. Re-enable once that is resolved.
list(REMOVE_ITEM MAPCREATOR_CORE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Algorithm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Transformation.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Optimization.cpp)

add_library(MapCreatorCore ${MAPCREATOR_CORE_SOURCES} ${MAPCREATOR_CORE_HEADERS})
add_library(MapCreator::Core ALIAS MapCreatorCore)

target_include_directories(MapCreatorCore PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/Core)

target_link_libraries(MapCreatorCore PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  glm::glm
  Boost::system Boost::filesystem Boost::serialization Boost::iostreams
  Qt5::Core Qt5::Widgets  # NOTE: Algorithm.cpp/Tracker pull in Qt (QMessageBox/QObject) — GUI coupling to revisit
  PkgConfig::LOG4CXX)

if(ENABLE_OPENCV_CONTRIB)
  target_compile_definitions(MapCreatorCore PUBLIC ENABLE_OPENCV_CONTRIB)
endif()

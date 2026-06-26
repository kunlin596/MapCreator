# MapCreatorHandler — image / tree data handlers.
file(GLOB MAPCREATOR_HANDLER_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB MAPCREATOR_HANDLER_HEADERS CONFIGURE_DEPENDS ${PROJECT_SOURCE_DIR}/include/Handler/*.h)

add_library(MapCreatorHandler ${MAPCREATOR_HANDLER_SOURCES} ${MAPCREATOR_HANDLER_HEADERS})
add_library(MapCreator::Handler ALIAS MapCreatorHandler)

target_include_directories(MapCreatorHandler PUBLIC
  ${PROJECT_SOURCE_DIR}/include/Handler)

target_link_libraries(MapCreatorHandler PUBLIC
  MapCreatorCore
  ${OpenCV_LIBS}
  Boost::system Boost::filesystem
  Qt5::Core Qt5::Concurrent)

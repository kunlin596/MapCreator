# MapCreatorGui — Qt OpenGL viewers and dialogs.
#
# The refactor moved the former Engine/*.cpp implementations here, so this single
# library now owns both the BasicViewer widgets (headers in include/BasicViewer,
# forms in ./form) and the dialogs (headers in include/Engine, forms in
# ../Engine/form). AUTOMOC/AUTOUIC/AUTORCC process the headers, .ui and .qrc.
file(GLOB MAPCREATOR_GUI_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB MAPCREATOR_GUI_HEADERS CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/include/BasicViewer/*.h
  ${PROJECT_SOURCE_DIR}/include/Engine/*.h)
file(GLOB MAPCREATOR_GUI_UI CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/form/*.ui
  ${PROJECT_SOURCE_DIR}/src/Engine/form/*.ui)
file(GLOB MAPCREATOR_GUI_QRC CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/rsc/*.qrc
  ${PROJECT_SOURCE_DIR}/src/Engine/rsc/*.qrc)

add_library(MapCreatorGui
  ${MAPCREATOR_GUI_SOURCES}
  ${MAPCREATOR_GUI_HEADERS}
  ${MAPCREATOR_GUI_UI}
  ${MAPCREATOR_GUI_QRC})
add_library(MapCreator::Gui ALIAS MapCreatorGui)

set_target_properties(MapCreatorGui PROPERTIES
  AUTOUIC_SEARCH_PATHS
    "${CMAKE_CURRENT_SOURCE_DIR}/form;${PROJECT_SOURCE_DIR}/src/Engine/form")

target_include_directories(MapCreatorGui PUBLIC
  ${PROJECT_SOURCE_DIR}/include/BasicViewer
  ${PROJECT_SOURCE_DIR}/include/Engine)

target_link_libraries(MapCreatorGui PUBLIC
  MapCreatorCore
  MapCreatorHandler
  ${OpenCV_LIBS}
  Boost::system Boost::filesystem
  Qt5::Widgets Qt5::Concurrent Qt5::OpenGL)

# Executables. Library usage requirements (includes, Qt, transitive deps) are
# inherited from the linked targets, so no manual include_directories needed.

add_executable(MapCreator MapCreator.cpp)
target_link_libraries(MapCreator PRIVATE MapCreatorGui)

add_executable(FrameViewer FrameViewer.cpp)
target_link_libraries(FrameViewer PRIVATE MapCreatorGui)

add_executable(dataset_converter dataset_converter.cpp)
target_link_libraries(dataset_converter PRIVATE MapCreatorCore Boost::program_options)

# Standalone placeholder utility.
add_executable(Viewer Viewer.cpp)

install(TARGETS MapCreator FrameViewer dataset_converter Viewer
  RUNTIME DESTINATION bin)

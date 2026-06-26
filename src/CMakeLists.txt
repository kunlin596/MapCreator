# Component libraries, declared in dependency order.
#   MapCreatorCore     – SLAM core algorithms              (src/Core)
#   MapCreatorHandler  – image/tree data handlers          (src/Streamer)
#   MapCreatorGui      – Qt viewers + dialogs              (src/Applications + Engine assets)
#   tools              – executables                       (src/Tools)
add_subdirectory(Core)
add_subdirectory(Streamer)
add_subdirectory(Applications)

if(MAPCREATOR_BUILD_TOOLS)
  add_subdirectory(Tools)
endif()

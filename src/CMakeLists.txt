# Component libraries, declared in dependency order.
#   MapCreatorCore     – SLAM core algorithms              (src/Core)
#   MapCreatorHandler  – image/tree data handlers          (src/Streamer)
#   MapCreatorGui      – Qt viewers + dialogs              (src/Applications + Engine assets)
#   tools              – executables                       (src/Tools)
add_subdirectory(Core)       # MapCreatorCore — SLAM library
add_subdirectory(Streamer)   # MapCreatorHandler — data handlers

# GUI + tool executables. Note: these compile against the redesigned APIs but
# are not runtime-verified here (no display); a few paths remain stubbed
# (calibrated converter, polymorphic tracker parameters) — see code comments.
option(MAPCREATOR_BUILD_APP "Build the Qt GUI and tool executables" ON)
if(MAPCREATOR_BUILD_APP)
  add_subdirectory(Applications)  # MapCreatorGui — viewers + dialogs
  if(MAPCREATOR_BUILD_TOOLS)
    add_subdirectory(Tools)
  endif()
endif()

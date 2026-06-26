# Component libraries, declared in dependency order.
#   MapCreatorCore     – SLAM core algorithms              (src/Core)
#   MapCreatorHandler  – image/tree data handlers          (src/Streamer)
#   MapCreatorGui      – Qt viewers + dialogs              (src/Applications + Engine assets)
#   tools              – executables                       (src/Tools)
add_subdirectory(Core)
add_subdirectory(Streamer)  # MapCreatorHandler — ported to the new KeyFrame/converter API

# The GUI / tool modules still target the pre-refactor APIs that were redesigned
# mid-refactor (mutable setter-based KeyFrame -> immutable point-cloud
# KeyFrame; converter ScreenToWorld/WorldToScreen -> Project/Unproject). They
# are being ported one module at a time; enable this once they compile.
option(MAPCREATOR_BUILD_APP "Build the (work-in-progress) GUI/tool modules" OFF)
if(MAPCREATOR_BUILD_APP)
  add_subdirectory(Applications)
  if(MAPCREATOR_BUILD_TOOLS)
    add_subdirectory(Tools)
  endif()
endif()

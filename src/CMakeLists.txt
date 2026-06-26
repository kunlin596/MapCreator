# Component libraries, declared in dependency order.
#   MapCreatorCore     – SLAM core algorithms              (src/Core)
#   MapCreatorHandler  – image/tree data handlers          (src/Streamer)
#   MapCreatorGui      – Qt viewers + dialogs              (src/Applications + Engine assets)
#   tools              – executables                       (src/Tools)
add_subdirectory(Core)

# The handler / GUI / tool modules still target the pre-refactor APIs that were
# redesigned mid-refactor (mutable setter-based KeyFrame -> immutable
# point-cloud-constructed KeyFrame; converter ScreenToWorld/WorldToScreen ->
# Project/Unproject). They are being ported one module at a time; enable this
# once a module compiles against the new API.
option(MAPCREATOR_BUILD_APP "Build the (work-in-progress) handler/GUI/tool modules" OFF)
if(MAPCREATOR_BUILD_APP)
  add_subdirectory(Streamer)
  add_subdirectory(Applications)
  if(MAPCREATOR_BUILD_TOOLS)
    add_subdirectory(Tools)
  endif()
endif()

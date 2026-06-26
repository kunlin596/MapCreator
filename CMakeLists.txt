cmake_minimum_required(VERSION 3.16)

project(MapCreator
  VERSION 0.1.0
  DESCRIPTION "Legacy RGB-D visual SLAM experiment toolset"
  LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
option(ENABLE_OPENCV_CONTRIB "Enable OpenCV contrib (xfeatures2d: FREAK/SIFT/SURF)" OFF)
option(MAPCREATOR_BUILD_TOOLS "Build the command-line / GUI tool executables" ON)
option(MAPCREATOR_BUILD_TESTS "Build the unit tests (GoogleTest)" ON)

# ---------------------------------------------------------------------------
# Language standard (applied to every target)
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
endif()

# ---------------------------------------------------------------------------
# Qt automatic tooling (moc / uic / rcc) — replaces manual qt5_wrap_* calls
# ---------------------------------------------------------------------------
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Predictable output locations
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ---------------------------------------------------------------------------
# Dependencies — found once here, consumed via imported targets downstream
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(glm REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem serialization iostreams program_options)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Concurrent OpenGL)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LOG4CXX REQUIRED IMPORTED_TARGET liblog4cxx)

message(STATUS "MapCreator: OpenCV ${OpenCV_VERSION}, Qt ${Qt5_VERSION}, "
               "contrib=${ENABLE_OPENCV_CONTRIB}")

add_subdirectory(src)

if(MAPCREATOR_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

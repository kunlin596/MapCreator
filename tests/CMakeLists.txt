# GoogleTest is pulled in via FetchContent so the test build is self-contained.
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# Helper: register a test executable for one source file, linked against the
# library under test, and discover its cases with CTest.
function(mapcreator_add_test name)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBS" ${ARGN})
  add_executable(${name} ${ARG_SOURCES})
  target_link_libraries(${name} PRIVATE ${ARG_LIBS} GTest::gtest_main)
  gtest_discover_tests(${name})
endfunction()

# ---- Per-module test suites (added one module at a time) ----
mapcreator_add_test(test_core_math
  SOURCES core/test_mymath.cpp
  LIBS MapCreatorCore)

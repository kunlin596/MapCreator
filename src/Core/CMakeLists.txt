cmake_minimum_required (VERSION 3.0)

project(MapCreatorCore VERSION 0.1.0 LANGUAGES CXX)

find_package(OpenCV REQUIRED)

aux_source_directory (${CMAKE_CURRENT_SOURCE_DIR} MapCreatorCore_SOURCE_FILES)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11 -Wall")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/usr/local/Qt5")

find_package (glm REQUIRED)
find_package (Boost COMPONENTS system filesystem serialization iostreams REQUIRED)

set(MapCreatorCore_INCLUDE_DIR ../../include)

include_directories (${MapCreatorCore_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}
                     ${MapCreator_INSTALL_INCLUDE_DIR})

add_library(mapcreatorcore_math SHARED ${CMAKE_CURRENT_SOURCE_DIR}/MyMath.cpp)
target_link_libraries(mapcreatorcore_math PUBLIC
                      ${OpenCV_LIBS} ${Boost_LIBRARIES} glm)

add_library(mapcreatorcore_utility SHARED ${CMAKE_CURRENT_SOURCE_DIR}/Utility.cpp)
target_link_libraries(mapcreatorcore_utility PUBLIC
                      ${OpenCV_LIBS} ${Boost_LIBRARIES} glm)
# target_link_libraries (MapCreatorCore
#                        PUBLIC
#                        ${LOG4CXX_LIBRARIES}
#                        ${OpenCV_LIBS}
#                        ${Boost_LIBRARIES}
#                        glm)

# install (TARGETS MapCreatorCore	DESTINATION "${CMAKE_INSTALL_PREFIX}/lib")

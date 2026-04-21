cmake_minimum_required(VERSION 3.21)

project(BallisticWorkspace LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(PREPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(osgEarth CONFIG REQUIRED)

set(BALLISTICS_MODULE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/modules/ballistics")
set(BALLISTICS_INCLUDE_DIR "${BALLISTICS_MODULE_DIR}/include")
set(BALLISTICS_SOURCES
    ${BALLISTICS_MODULE_DIR}/src/config.cpp
    ${BALLISTICS_MODULE_DIR}/src/defaults.cpp
    ${BALLISTICS_MODULE_DIR}/src/dynamics.cpp
    ${BALLISTICS_MODULE_DIR}/src/interpolation.cpp
    ${BALLISTICS_MODULE_DIR}/src/live_writer.cpp
    ${BALLISTICS_MODULE_DIR}/src/logging.cpp
    ${BALLISTICS_MODULE_DIR}/src/rk4.cpp
    ${BALLISTICS_MODULE_DIR}/src/search.cpp
    ${BALLISTICS_MODULE_DIR}/src/solver.cpp
    ${BALLISTICS_MODULE_DIR}/src/trajectory.cpp
)

add_library(ballistics STATIC ${BALLISTICS_SOURCES})
target_include_directories(ballistics PUBLIC "${BALLISTICS_INCLUDE_DIR}")
target_link_libraries(ballistics PUBLIC osgEarth::osgEarth)

add_executable(ballistic_solver_cli apps/solver_cli/src/main.cpp)
target_link_libraries(ballistic_solver_cli PRIVATE ballistics)

add_executable(ballistic_osgearth apps/osgearth_viewer/src/main.cpp)
target_include_directories(ballistic_osgearth PRIVATE "${BALLISTICS_INCLUDE_DIR}")
target_link_libraries(
    ballistic_osgearth
    PRIVATE
        ballistics
        osgEarth::osgEarth
        osgEarth::osgEarthImGui
)

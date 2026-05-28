cmake_minimum_required(VERSION 3.16)

project(GtsamLocalizerStarter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)

set(NTCORE_INCLUDE_DIR "" CACHE PATH "Path to WPILib ntcore/networktables headers")
set(NTCORE_LIBRARY "" CACHE FILEPATH "Path to libntcore")

add_executable(gtsam_localizer
    src/main.cc
)

target_include_directories(gtsam_localizer PRIVATE
    ${NTCORE_INCLUDE_DIR}
)

target_link_libraries(gtsam_localizer
    gtsam
    ${NTCORE_LIBRARY}
)
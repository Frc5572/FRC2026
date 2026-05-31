cmake_minimum_required(VERSION 3.16)

project(GtsamLocalizerStarter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GTSAM REQUIRED)

set(WPILIB_YEAR "2026" CACHE STRING "WPILib install year")
set(WPILIB_VERSION "2026.2.1" CACHE STRING "WPILib artifact version")
set(WPILIB_ROOT "$ENV{HOME}/wpilib/${WPILIB_YEAR}" CACHE PATH "WPILib install root")
set(WPILIB_MAVEN "${WPILIB_ROOT}/maven/edu/wpi/first")
set(WPILIB_EXTRACT_DIR "${CMAKE_BINARY_DIR}/wpilib")

function(add_wpilib_cpp_library target group artifact lib_name)
    set(base_zip "${WPILIB_MAVEN}/${group}/${artifact}-cpp/${WPILIB_VERSION}/${artifact}-cpp-${WPILIB_VERSION}")
    set(headers_zip "${base_zip}-headers.zip")
    set(native_zip "${base_zip}-osxuniversal.zip")
    set(include_dir "${WPILIB_EXTRACT_DIR}/${artifact}/include")
    set(native_dir "${WPILIB_EXTRACT_DIR}/${artifact}/native")

    if(NOT EXISTS "${headers_zip}")
        message(FATAL_ERROR "Missing WPILib headers archive: ${headers_zip}")
    endif()
    if(NOT EXISTS "${native_zip}")
        message(FATAL_ERROR "Missing WPILib native archive: ${native_zip}")
    endif()

    file(MAKE_DIRECTORY "${include_dir}" "${native_dir}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xf "${headers_zip}"
        WORKING_DIRECTORY "${include_dir}"
        COMMAND_ERROR_IS_FATAL ANY
    )
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E tar xf "${native_zip}"
        WORKING_DIRECTORY "${native_dir}"
        COMMAND_ERROR_IS_FATAL ANY
    )

    add_library(${target} SHARED IMPORTED)
    set_target_properties(${target} PROPERTIES
        IMPORTED_LOCATION "${native_dir}/osx/universal/shared/lib${lib_name}.dylib"
        INTERFACE_INCLUDE_DIRECTORIES "${include_dir}"
    )
endfunction()

add_wpilib_cpp_library(wpiutil_wpilib wpiutil wpiutil wpiutil)
add_wpilib_cpp_library(wpinet_wpilib wpinet wpinet wpinet)
add_wpilib_cpp_library(ntcore_wpilib ntcore ntcore ntcore)

target_link_libraries(wpinet_wpilib INTERFACE wpiutil_wpilib)
target_link_libraries(ntcore_wpilib INTERFACE wpinet_wpilib wpiutil_wpilib)

add_executable(gtsam_localizer
    src/main.cc
)

target_link_libraries(gtsam_localizer
    gtsam
    ntcore_wpilib
)

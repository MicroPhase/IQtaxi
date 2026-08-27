cmake_minimum_required(VERSION 3.11)
project(microphase_iq_taxi LANGUAGES CXX C)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
                 Debug Release RelWithDebInfo MinSizeRel)
endif()

option(ENABLE_E100 "Build E100 module" ON)
option(ENABLE_E200 "Build E200 module" ON)
option(ENABLE_GNURADIO_IQTAXI "Build GNU Radio IQTAXI RX/TX blocks" ON)
option(ENABLE_WIN "Build Windows static libraries (MXE)" OFF)
option(ENABLE_SDRANGEL_IQTAXI "Build SDRangel IQTAXI input plugin (E100/E200/E206)" OFF)
option(ENABLE_SDRPP_IQTAXI "Build SDR++ IQTAXI source plugin (E100/E200/E206)" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SDR_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/)
message(STATUS "SDR头文件目录: ${SDR_INCLUDE_DIR}")

add_subdirectory(src/core)
add_subdirectory(src/driver)

if(ENABLE_WIN)
    add_subdirectory(src/example)
    add_subdirectory(src/HDSDR)
else()
    add_subdirectory(src/example)
    add_subdirectory(src/soapysdr)
    add_subdirectory(src/UHD)
    add_subdirectory(src/plugin)
    if(ENABLE_GNURADIO_IQTAXI)
        add_subdirectory(src/gnuradio)
    endif()
    if(ENABLE_SDRANGEL_IQTAXI)
        add_subdirectory(src/sdrangel)
    endif()
    if(ENABLE_SDRPP_IQTAXI)
        add_subdirectory(src/sdr++)
    endif()

# CMake installs are recorded in install_manifest.txt. Provide the matching
# removal target so users can run `sudo make uninstall` from the build tree.
configure_file(
    "${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall
    COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
    COMMENT "Removing installed IQTAXI files")
endif()

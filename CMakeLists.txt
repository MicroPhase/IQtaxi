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
option(ENABLE_SDRANGEL_IQTAXI "Build SDRangel IQTAXI input plugin (E100/E200/E206)" ON)
option(ENABLE_SDRPP_IQTAXI "Build SDR++ IQTAXI source plugin (E100/E200/E206)" ON)

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

    if(ENABLE_SDRPP_IQTAXI)
        find_library(SDRPP_CORE_LIBRARY sdrpp_core)
        set(SDRPP_ROOT "/home/kang/packages/SDRPlusPlus" CACHE PATH
            "SDR++ source tree (needs core/src headers for OOT plugins)")
        if(SDRPP_CORE_LIBRARY AND EXISTS "${SDRPP_ROOT}/core/src/module.h")
            message(STATUS "SDR++ found (${SDRPP_CORE_LIBRARY}), building iqtaxi_source")
            add_subdirectory(src/sdr++)
        else()
            message(STATUS
                "SDR++ not found (need libsdrpp_core + SDRPP_ROOT/core/src/module.h); "
                "skipping iqtaxi_source. Set -DSDRPP_ROOT=... or -DENABLE_SDRPP_IQTAXI=OFF")
        endif()
    endif()

    if(ENABLE_SDRANGEL_IQTAXI)
        set(SDRANGEL_SOURCE_DIR "/home/kang/packages/sdrangel" CACHE PATH
            "SDRangel source tree path (headers)")
        if(EXISTS "${SDRANGEL_SOURCE_DIR}/sdrbase/plugin/plugininterface.h")
            message(STATUS "SDRangel found (${SDRANGEL_SOURCE_DIR}), building iqtaxiinput")
            add_subdirectory(src/sdrangel)
        else()
            message(STATUS
                "SDRangel headers not found at ${SDRANGEL_SOURCE_DIR}; "
                "skipping iqtaxiinput. Set -DSDRANGEL_SOURCE_DIR=... or -DENABLE_SDRANGEL_IQTAXI=OFF")
        endif()
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

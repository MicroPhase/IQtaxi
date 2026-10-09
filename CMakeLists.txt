cmake_minimum_required(VERSION 3.11)
project(iqtaxi LANGUAGES CXX C)

include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
                 Debug Release RelWithDebInfo MinSizeRel)
endif()

option(ENABLE_E100 "Build E100 module" ON)
option(ENABLE_E200 "Build E200 module" ON)
option(ENABLE_WIN "Build Windows static libraries (MXE)" OFF)

# Host application integrations are opt-in so the native GUI/API build does
# not require third-party SDR application SDKs.
option(ENABLE_SOAPYSDR_IQTAXI "Build SoapySDR IQTAXI plugin" OFF)
option(ENABLE_UHD_IQTAXI "Build UHD IQTAXI plugin" OFF)
option(ENABLE_GNURADIO_IQTAXI "Build GNU Radio IQTAXI RX/TX blocks" OFF)
option(ENABLE_SDRANGEL_IQTAXI "Build SDRangel IQTAXI input plugin (E100/E200/E206)" OFF)
option(ENABLE_SDRPP_IQTAXI "Build SDR++ IQTAXI source plugin (E100/E200/E206)" OFF)
option(ENABLE_HDSDR_IQTAXI "Build HDSDR IQTAXI ExtIO plugin (requires ENABLE_WIN)" OFF)

message(STATUS "")
message(STATUS "========== IQTAXI Build Configuration ==========")
message(STATUS "Base features: core library, device driver, GUI, examples and utilities")
message(STATUS "Plugin status (OFF = not built, ON = built):")
message(STATUS "  SoapySDR : ${ENABLE_SOAPYSDR_IQTAXI}")
message(STATUS "  UHD      : ${ENABLE_UHD_IQTAXI}")
message(STATUS "  GNU Radio: ${ENABLE_GNURADIO_IQTAXI}")
message(STATUS "  SDR++    : ${ENABLE_SDRPP_IQTAXI}")
message(STATUS "  SDRangel : ${ENABLE_SDRANGEL_IQTAXI}")
message(STATUS "  HDSDR    : ${ENABLE_HDSDR_IQTAXI} (also requires ENABLE_WIN=ON)")
message(STATUS "To enable a plugin, re-run CMake from this build directory with one of:")
message(STATUS "  SoapySDR : cmake ../ -DENABLE_SOAPYSDR_IQTAXI=ON")
message(STATUS "  UHD      : cmake ../ -DENABLE_UHD_IQTAXI=ON")
message(STATUS "  GNU Radio: cmake ../ -DENABLE_GNURADIO_IQTAXI=ON")
message(STATUS "  SDR++    : cmake ../ -DENABLE_SDRPP_IQTAXI=ON -DSDRPP_ROOT=/path/to/SDRPlusPlus")
message(STATUS "  SDRangel : cmake ../ -DENABLE_SDRANGEL_IQTAXI=ON -DSDRANGEL_SOURCE_DIR=/path/to/sdrangel")
message(STATUS "  HDSDR    : cmake ../ -DENABLE_WIN=ON -DENABLE_HDSDR_IQTAXI=ON (Windows/MXE)")
message(STATUS "Multiple -D options may be combined. Plugin dependencies must be installed separately.")
message(STATUS "================================================")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SDR_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/)
message(STATUS "SDR include directory: ${SDR_INCLUDE_DIR}")

add_subdirectory(src/core)
add_subdirectory(src/driver)

if(ENABLE_WIN)
    add_subdirectory(src/example)
    if(ENABLE_HDSDR_IQTAXI)
        add_subdirectory(src/HDSDR)
    endif()
else()
    add_subdirectory(src/example)
    if(ENABLE_SOAPYSDR_IQTAXI)
        add_subdirectory(src/soapysdr)
    endif()
    if(ENABLE_UHD_IQTAXI)
        add_subdirectory(src/UHD)
    endif()
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

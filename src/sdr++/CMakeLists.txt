cmake_minimum_required(VERSION 3.13)

# Built as part of MICROPHASE_IQ_TAXI, or standalone from this directory.
if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(iqtaxi_source LANGUAGES CXX)
endif ()

set(SDRPP_ROOT "/home/kang/packages/SDRPlusPlus" CACHE PATH
    "SDR++ source tree (needs core/src headers for OOT plugins)")
if (NOT SDRPP_ROOT AND DEFINED ENV{SDRPP_ROOT})
    set(SDRPP_ROOT "$ENV{SDRPP_ROOT}" CACHE PATH
        "SDR++ source tree (needs core/src headers for OOT plugins)" FORCE)
endif ()
set(SDRPP_PLUGIN_INSTALL_DIR "lib/sdrpp/plugins" CACHE STRING
    "Install destination for the SDR++ plugin (under CMAKE_INSTALL_PREFIX)")

if (DEFINED SDR_INCLUDE_DIR)
    set(_IQTAXI_ROOT "${SDR_INCLUDE_DIR}")
else ()
    get_filename_component(_IQTAXI_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
endif ()

set(IQTAXI_ROOT "${_IQTAXI_ROOT}" CACHE PATH
    "MICROPHASE_IQ_TAXI source root (for include/sdr/...)" FORCE)
set(IQTAXI_LIB_DIR "/usr/local/lib" CACHE PATH
    "Directory containing libsdr_core.so and libsdr_driver.so")

if (NOT SDRPP_CORE_LIBRARY)
    find_library(SDRPP_CORE_LIBRARY sdrpp_core)
endif ()
if (NOT SDRPP_CORE_LIBRARY)
    message(FATAL_ERROR "iqtaxi_source: libsdrpp_core not found (is SDR++ installed?)")
endif ()

set(_SDRPP_CORE_SRC "${SDRPP_ROOT}/core/src")
if (NOT EXISTS "${_SDRPP_CORE_SRC}/module.h")
    message(FATAL_ERROR
        "iqtaxi_source: SDR++ headers not found at ${_SDRPP_CORE_SRC}/module.h\n"
        "Set -DSDRPP_ROOT=/path/to/SDRPlusPlus or export SDRPP_ROOT")
endif ()

file(GLOB IQTAXI_SDRPP_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)
if (NOT IQTAXI_SDRPP_SRC)
    message(FATAL_ERROR "iqtaxi_source: no .cpp sources found under ${CMAKE_CURRENT_SOURCE_DIR}")
endif ()

add_library(iqtaxi_source SHARED ${IQTAXI_SDRPP_SRC})
set_target_properties(iqtaxi_source PROPERTIES PREFIX "" OUTPUT_NAME "iqtaxi_source")
target_compile_features(iqtaxi_source PRIVATE cxx_std_17)

target_include_directories(iqtaxi_source PRIVATE
    "${IQTAXI_ROOT}"
    "${_SDRPP_CORE_SRC}"
    "${_SDRPP_CORE_SRC}/imgui"
)

if (TARGET sdr_core AND TARGET sdr_driver)
    target_link_libraries(iqtaxi_source PRIVATE
        ${SDRPP_CORE_LIBRARY}
        sdr_core
        sdr_driver
        pthread
    )
else ()
    find_library(SDR_CORE_LIBRARY sdr_core REQUIRED
        HINTS "${IQTAXI_LIB_DIR}" "${IQTAXI_ROOT}/build/src/core")
    find_library(SDR_DRIVER_LIBRARY sdr_driver REQUIRED
        HINTS "${IQTAXI_LIB_DIR}" "${IQTAXI_ROOT}/build/src/driver")
    target_link_libraries(iqtaxi_source PRIVATE
        ${SDRPP_CORE_LIBRARY}
        ${SDR_CORE_LIBRARY}
        ${SDR_DRIVER_LIBRARY}
        pthread
    )
endif ()

# Default prefix /usr/local → /usr/local/lib/sdrpp/plugins
install(TARGETS iqtaxi_source
    LIBRARY DESTINATION "${SDRPP_PLUGIN_INSTALL_DIR}"
    RUNTIME DESTINATION "${SDRPP_PLUGIN_INSTALL_DIR}"
)

message(STATUS "iqtaxi_source: SDRPP_ROOT=${SDRPP_ROOT}")
message(STATUS "iqtaxi_source: IQTAXI_ROOT=${IQTAXI_ROOT}")
message(STATUS "iqtaxi_source: sdrpp_core=${SDRPP_CORE_LIBRARY}")
message(STATUS "iqtaxi_source: install → \${CMAKE_INSTALL_PREFIX}/${SDRPP_PLUGIN_INSTALL_DIR}")

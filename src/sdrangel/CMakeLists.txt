cmake_minimum_required(VERSION 3.13)
project(sdrangel_iqtaxi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_SDRANGEL_IQTAXI_SKELETON "Build IQTAXI backend skeleton" ON)
option(BUILD_SDRANGEL_IQTAXI_PLUGIN "Build SDRangel plugin interface target" ON)

set(SDRANGEL_SOURCE_DIR "" CACHE PATH
    "SDRangel source tree path (headers)")
if(NOT SDRANGEL_SOURCE_DIR AND DEFINED ENV{SDRANGEL_SOURCE_DIR})
    set(SDRANGEL_SOURCE_DIR "$ENV{SDRANGEL_SOURCE_DIR}" CACHE PATH
        "SDRangel source tree path (headers)" FORCE)
endif()
# System SDRangel puts libs next to plugins: /usr/local/lib/sdrangel/{libsdrbase.so,plugins/}
set(SDRANGEL_BASE_LIB_DIR "/usr/local/lib/sdrangel" CACHE PATH
    "Directory containing libsdrbase.so / libsdrgui.so")
set(SDRANGEL_PLUGIN_INSTALL_DIR
    "/usr/local/lib/sdrangel/plugins"
    CACHE PATH "Install directory for libinputiqtaxi.so")

if(BUILD_SDRANGEL_IQTAXI_SKELETON)
    add_library(sdrangel_iqtaxi_skeleton STATIC
        iqtaxi_settings.cpp
        iqtaxi_worker.cpp
        iqtaxi_backend.cpp
    )

    target_include_directories(sdrangel_iqtaxi_skeleton PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
    )

    target_link_libraries(sdrangel_iqtaxi_skeleton PUBLIC
        sdr_driver
        sdr_core
        pthread
    )
endif()

if(BUILD_SDRANGEL_IQTAXI_PLUGIN)
    if(NOT EXISTS "${SDRANGEL_SOURCE_DIR}/sdrbase/plugin/plugininterface.h")
        message(FATAL_ERROR
            "SDRangel headers not found. Set -DSDRANGEL_SOURCE_DIR=/path/to/sdrangel")
    endif()

    find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

    add_library(iqtaxiinput MODULE
        iqtaxiplugin.cpp
        iqtaxiinput.cpp
        iqtaxiwebapiadapter.cpp
        iqtaxi_settings.cpp
        iqtaxi_backend.cpp
        iqtaxi_worker.cpp
    )
    set_target_properties(iqtaxiinput PROPERTIES AUTOMOC ON)

    if(NOT SERVER_MODE)
        target_sources(iqtaxiinput PRIVATE
            iqtaxigui.cpp
        )
    endif()

    target_include_directories(iqtaxiinput PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        ${SDRANGEL_SOURCE_DIR}/sdrbase
        ${SDRANGEL_SOURCE_DIR}/sdrgui
        ${SDRANGEL_SOURCE_DIR}/exports
        ${SDRANGEL_SOURCE_DIR}/swagger/sdrangel/code/qt5/client
    )

    # Must match SDRangel build: default RX_SAMPLE_24BIT=ON makes FixReal=qint32.
    # Without this, Sample layout is 4B vs 8B and FIFO/waterfall data is garbage.
    target_compile_definitions(iqtaxiinput PRIVATE SDR_RX_SAMPLE_24BIT)

    if(TARGET sdrbase)
        set(_SDRANGEL_BASE_LIB sdrbase)
    else()
        find_library(_SDRANGEL_BASE_LIB NAMES sdrbase
            HINTS
                ${SDRANGEL_BASE_LIB_DIR}
                /usr/local/lib/sdrangel
                /usr/local/lib
                /usr/lib
                ${SDRANGEL_SOURCE_DIR}/build/lib
        )
        if(NOT _SDRANGEL_BASE_LIB)
            message(FATAL_ERROR
                "Could not find libsdrbase (SDRangel base library).\n"
                "Set -DSDRANGEL_BASE_LIB_DIR=/usr/local/lib/sdrangel")
        endif()
    endif()

    if(NOT SERVER_MODE)
        if(TARGET sdrgui)
            set(_SDRANGEL_GUI_LIB sdrgui)
        else()
            find_library(_SDRANGEL_GUI_LIB NAMES sdrgui
                HINTS
                    ${SDRANGEL_BASE_LIB_DIR}
                    /usr/local/lib/sdrangel
                    /usr/local/lib
                    /usr/lib
                    ${SDRANGEL_SOURCE_DIR}/build/lib
            )
            if(NOT _SDRANGEL_GUI_LIB)
                message(FATAL_ERROR
                    "Could not find libsdrgui.\n"
                    "Set -DSDRANGEL_BASE_LIB_DIR=/usr/local/lib/sdrangel")
            endif()
        endif()
    endif()

    target_link_libraries(iqtaxiinput PRIVATE
        Qt5::Core
        Qt5::Widgets
        ${_SDRANGEL_BASE_LIB}
        ${_SDRANGEL_GUI_LIB}
        sdr_driver
        sdr_core
        pthread
    )

    # System layout: .../sdrangel/plugins/*.so -> libs at .../sdrangel/
    set_target_properties(iqtaxiinput PROPERTIES
        PREFIX "lib"
        OUTPUT_NAME "inputiqtaxi"
        BUILD_RPATH "/usr/local/lib/sdrangel;$ORIGIN/.."
        INSTALL_RPATH "/usr/local/lib/sdrangel;$ORIGIN/.."
    )

    message(STATUS "iqtaxiinput: sdrbase=${_SDRANGEL_BASE_LIB}")
    message(STATUS "iqtaxiinput: install → ${SDRANGEL_PLUGIN_INSTALL_DIR}")

    install(TARGETS iqtaxiinput
        COMPONENT SDRangelIQTAXIPlugin
        LIBRARY DESTINATION ${SDRANGEL_PLUGIN_INSTALL_DIR}
    )
endif()

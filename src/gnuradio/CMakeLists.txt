if(WIN32)
    return()
endif()

include(GNUInstallDirs)

find_package(Gnuradio REQUIRED COMPONENTS runtime)

include(GrPython)
include(GrSwig OPTIONAL RESULT_VARIABLE IQTAXI_GR_SWIG_INCLUDED)
include(GrPybind OPTIONAL RESULT_VARIABLE IQTAXI_GR_PYBIND_INCLUDED)

set(IQTAXI_GR_HAS_SWIG_BINDINGS OFF)
if(IQTAXI_GR_SWIG_INCLUDED AND COMMAND GR_SWIG_MAKE)
    find_package(SWIG REQUIRED)
    set(IQTAXI_GR_HAS_SWIG_BINDINGS ON)
endif()

set(IQTAXI_GR_HAS_PYBIND_HELPERS OFF)
if(IQTAXI_GR_PYBIND_INCLUDED AND COMMAND GR_PYBIND_MAKE_OOT)
    set(IQTAXI_GR_HAS_PYBIND_HELPERS ON)
endif()

if(DEFINED Gnuradio_VERSION)
    set(IQTAXI_GR_VERSION_TEXT "${Gnuradio_VERSION}")
elseif(DEFINED GNURADIO_VERSION)
    set(IQTAXI_GR_VERSION_TEXT "${GNURADIO_VERSION}")
else()
    set(IQTAXI_GR_VERSION_TEXT "unknown")
endif()

add_library(gnuradio-iqtaxi SHARED
    lib/device_context.cpp
    lib/rx_source_impl.cpp
    lib/tx_sink_impl.cpp
)

target_compile_definitions(gnuradio-iqtaxi PRIVATE gnuradio_iqtaxi_EXPORTS)
target_compile_options(gnuradio-iqtaxi PRIVATE -fPIC)
target_include_directories(gnuradio-iqtaxi
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${SDR_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}
)
target_link_libraries(gnuradio-iqtaxi
    PUBLIC
        gnuradio::gnuradio-runtime
    PRIVATE
        sdr_driver
        pthread
)
set_target_properties(gnuradio-iqtaxi PROPERTIES
    OUTPUT_NAME gnuradio-iqtaxi
)

install(TARGETS gnuradio-iqtaxi
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/gnuradio
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(IQTAXI_GR_HAS_SWIG_BINDINGS)
    message(STATUS "GNU Radio ${IQTAXI_GR_VERSION_TEXT}: building IQTAXI SWIG bindings")
    set(GR_SWIG_INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${SDR_INCLUDE_DIR}
        ${CMAKE_SOURCE_DIR}
        $<TARGET_PROPERTY:gnuradio::runtime_swig,INTERFACE_INCLUDE_DIRECTORIES>
    )
    set(GR_SWIG_LIBRARIES gnuradio-iqtaxi gnuradio::gnuradio-runtime)
    set(GR_SWIG_TARGET_DEPS gnuradio-iqtaxi)
    set(GR_SWIG_FLAGS "")
    GR_SWIG_MAKE(iqtaxi_swig swig/iqtaxi_swig.i)
    install(TARGETS iqtaxi_swig
        DESTINATION ${GR_PYTHON_DIR}/gnuradio/iqtaxi
    )
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/iqtaxi_swig.py
        python/iqtaxi/__init__.py
        DESTINATION ${GR_PYTHON_DIR}/gnuradio/iqtaxi
    )

    install(FILES swig/iqtaxi_swig.i
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gnuradio/iqtaxi/swig
    )
elseif(IQTAXI_GR_HAS_PYBIND_HELPERS)
    message(STATUS "GNU Radio ${IQTAXI_GR_VERSION_TEXT}: building IQTAXI pybind11 bindings")
    pybind11_add_module(iqtaxi_python
        python/bindings/python_bindings.cc
    )
    target_include_directories(iqtaxi_python
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
            ${SDR_INCLUDE_DIR}
            ${CMAKE_SOURCE_DIR}
    )
    target_link_libraries(iqtaxi_python
        PRIVATE
            gnuradio-iqtaxi
            gnuradio::gnuradio-runtime
    )
    install(TARGETS iqtaxi_python
        LIBRARY DESTINATION ${GR_PYTHON_DIR}/gnuradio/iqtaxi
    )
    install(FILES python/iqtaxi/__init__.py
        DESTINATION ${GR_PYTHON_DIR}/gnuradio/iqtaxi
    )
else()
    message(WARNING
        "GNU Radio ${IQTAXI_GR_VERSION_TEXT}: neither GrSwig nor GrPybind helpers were found. "
        "Building the C++ IQTAXI block library and GRC block descriptors only."
    )
endif()

if(NOT GRC_BLOCKS_DIR)
    set(GRC_BLOCKS_DIR share/gnuradio/grc/blocks)
endif()
install(FILES
    grc/iqtaxi_rx_source.block.yml
    grc/iqtaxi_tx_sink.block.yml
    DESTINATION ${GRC_BLOCKS_DIR}
)
install(PROGRAMS m300_rx_capture.py
    DESTINATION ${CMAKE_INSTALL_BINDIR}
    RENAME iqtaxi_m300_rx_capture
)

if(ENABLE_WIN)
    set(TRANSPORT_SOURCES
        device.cpp
        IQTAXI/iqtaxi_udp_impl.cpp
        IQTAXI/iqtaxi_udp_discover.cpp
        E100/e100_impl.cpp
        E200/e200_impl.cpp
        E206/e206_impl.cpp
        transport/local_ctrl.cpp
    )
    add_library(sdr_driver STATIC ${TRANSPORT_SOURCES})
else()
    file(GLOB_RECURSE TRANSPORT_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    )
    add_library(sdr_driver SHARED ${TRANSPORT_SOURCES})
    set_target_properties(sdr_driver PROPERTIES
        VERSION 1.0.0
        SOVERSION 1
    )
endif()

set(M300_NOOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/M300/noos_ad9361)
set(M300_AD9361_DIR ${M300_NOOS_DIR}/vendor/ad9361)
set(M300_NOOS_COMMON_DIR ${M300_NOOS_DIR}/vendor/no_os)

set(M300_AD9361_SOURCES
    ${M300_AD9361_DIR}/ad9361.c
    ${M300_AD9361_DIR}/ad9361_api.c
    ${M300_AD9361_DIR}/ad9361_conv.c
    ${M300_AD9361_DIR}/ad9361_util.c
    ${M300_AD9361_DIR}/delay.c
    ${M300_NOOS_COMMON_DIR}/no_os_gpio.c
    ${M300_NOOS_COMMON_DIR}/no_os_util.c
    ${M300_NOOS_DIR}/m300_axi_adc_core.c
    ${M300_NOOS_DIR}/m300_axi_dac_core.c
    ${M300_NOOS_DIR}/m300_ad9361_defaults.c
)

if(NOT ENABLE_WIN)
    target_sources(sdr_driver PRIVATE ${M300_AD9361_SOURCES})
endif()

target_link_libraries(sdr_driver PUBLIC 
                    sdr_core
)

if(NOT ENABLE_WIN)
    target_link_libraries(sdr_driver PRIVATE pthread)
    enable_testing()
    find_package(Threads REQUIRED)
    add_executable(iqtaxi_clock_reference_test ../../tests/clock_reference_test.cpp)
    target_include_directories(iqtaxi_clock_reference_test PRIVATE ${SDR_INCLUDE_DIR})
    target_link_libraries(iqtaxi_clock_reference_test PRIVATE Threads::Threads)
    add_test(NAME iqtaxi_clock_reference COMMAND iqtaxi_clock_reference_test)
endif()

target_compile_definitions(sdr_driver PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:__STDC_VERSION__=199901L>
)
                    
target_include_directories(sdr_driver PUBLIC
        ${SDR_INCLUDE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/M300
        ${M300_NOOS_DIR}
        ${M300_AD9361_DIR}
        ${M300_NOOS_COMMON_DIR}
)

if(ENABLE_WIN)

else()
    install(TARGETS sdr_driver
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT RuntimeAPI
    )
    install(DIRECTORY ../../include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iq_taxi
        COMPONENT RuntimeAPI)
    # External programs include iq_taxi/src/driver/*.hpp. Those headers use
    # ../../../include/sdr/..., which resolves when iq_taxi/include points at
    # the installed iq_taxi tree (where the public headers live).
    install(DIRECTORY
            E100
            E200
            E206
            IQTAXI
            transport
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iq_taxi/src/driver
        COMPONENT RuntimeAPI
        FILES_MATCHING
            PATTERN "*.hpp"
            PATTERN "*.h")
    install(CODE "
        set(_iqtaxi_include \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/iq_taxi/include\")
        if(NOT EXISTS \"\${_iqtaxi_include}\")
            execute_process(
                COMMAND \"${CMAKE_COMMAND}\" -E create_symlink . \"\${_iqtaxi_include}\"
                RESULT_VARIABLE _iqtaxi_include_link_result)
            if(NOT _iqtaxi_include_link_result EQUAL 0)
                message(FATAL_ERROR \"Failed to create \${_iqtaxi_include}\")
            endif()
        endif()
    " COMPONENT RuntimeAPI)
endif()

if(ENABLE_WIN)
    set(TRANSPORT_SOURCES
        device.cpp
        IQTAXI/iqtaxi_udp_impl.cpp
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
        LIBRARY DESTINATION /usr/local/lib
    )
    install(DIRECTORY ../../include/ DESTINATION include/iq_taxi)
endif()

set(SDR_CORE
        chdr_header.cpp
        time_spec.cpp
        zero_copy.cpp
        udp_zero_copy.cpp
)
if(NOT ENABLE_WIN)
    list(APPEND SDR_CORE xdma_zero_copy.cpp)
endif()

if(ENABLE_WIN)
        add_library(sdr_core STATIC ${SDR_CORE})
else()
        add_library(sdr_core SHARED ${SDR_CORE})
        set_target_properties(sdr_core PROPERTIES
                VERSION 1.0.0
                SOVERSION 1
        )
endif()

target_include_directories(sdr_core PUBLIC
        ${SDR_INCLUDE_DIR}
)

get_target_property(inc_dirs sdr_core INCLUDE_DIRECTORIES)
message(STATUS "核心库包含路径: ${inc_dirs}")

if(ENABLE_WIN)
        target_link_libraries(sdr_core PUBLIC
                opengl32
                setupapi
                advapi32
                iphlpapi
                ws2_32
        )
else()
        target_compile_options(sdr_core PUBLIC -fPIC)
        install(TARGETS sdr_core
                LIBRARY DESTINATION /usr/local/lib
        )
endif()

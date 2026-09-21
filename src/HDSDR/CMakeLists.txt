if(NOT ENABLE_WIN)
    message(STATUS "Skipping HDSDR ExtIO_ANTSDR (ENABLE_WIN=OFF)")
    return()
endif()

# HDSDR host adapter (ExtIO DLL). Keep OUTPUT_NAME ExtIO_* so HDSDR can discover it.
add_library(ExtIO_ANTSDR SHARED
    ExtIO_ANTSDR.cpp
)

target_include_directories(ExtIO_ANTSDR PRIVATE
    ${SDR_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ExtIO_ANTSDR PRIVATE
    sdr_core
    sdr_driver
)

# Keep ExtIO entry points unmangled for HDSDR.
set_target_properties(ExtIO_ANTSDR PROPERTIES
    PREFIX ""
    OUTPUT_NAME "ExtIO_ANTSDR"
    WINDOWS_EXPORT_ALL_SYMBOLS OFF
)

if(MINGW OR CMAKE_CROSSCOMPILING)
    target_link_options(ExtIO_ANTSDR PRIVATE
        "-Wl,--enable-stdcall-fix"
        "-static-libgcc"
        "-static-libstdc++"
    )
endif()

# .def keeps undecorated export names for MinGW/MSVC.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ExtIO_ANTSDR.def")
    if(MSVC)
        target_sources(ExtIO_ANTSDR PRIVATE ExtIO_ANTSDR.def)
    else()
        target_link_options(ExtIO_ANTSDR PRIVATE
            "LINKER:${CMAKE_CURRENT_SOURCE_DIR}/ExtIO_ANTSDR.def"
        )
    endif()
endif()

message(STATUS "HDSDR: building ExtIO_ANTSDR.dll via native IQTAXI API")

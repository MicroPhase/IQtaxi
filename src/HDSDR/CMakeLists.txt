if(NOT ENABLE_WIN)
    message(STATUS "Skipping HDSDR ExtIO_MicroPhase (ENABLE_WIN=OFF)")
    return()
endif()

# HDSDR host adapter (ExtIO DLL). Keep OUTPUT_NAME ExtIO_* so HDSDR can discover it.
add_library(ExtIO_MicroPhase SHARED
    ExtIO_MicroPhase.cpp
)

target_include_directories(ExtIO_MicroPhase PRIVATE
    ${SDR_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ExtIO_MicroPhase PRIVATE
    sdr_core
    sdr_driver
)

# Keep ExtIO entry points unmangled for HDSDR.
set_target_properties(ExtIO_MicroPhase PROPERTIES
    PREFIX ""
    OUTPUT_NAME "ExtIO_MicroPhase"
    WINDOWS_EXPORT_ALL_SYMBOLS OFF
)

if(MINGW OR CMAKE_CROSSCOMPILING)
    target_link_options(ExtIO_MicroPhase PRIVATE
        "-Wl,--enable-stdcall-fix"
        "-static-libgcc"
        "-static-libstdc++"
    )
endif()

# .def keeps undecorated export names for MinGW/MSVC.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ExtIO_MicroPhase.def")
    if(MSVC)
        target_sources(ExtIO_MicroPhase PRIVATE ExtIO_MicroPhase.def)
    else()
        target_link_options(ExtIO_MicroPhase PRIVATE
            "LINKER:${CMAKE_CURRENT_SOURCE_DIR}/ExtIO_MicroPhase.def"
        )
    endif()
endif()

message(STATUS "HDSDR: building ExtIO_MicroPhase.dll via native IQTAXI API")

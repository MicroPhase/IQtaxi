add_executable(MicroSDRUtil
    MicroSDRUtil.cpp
)

target_include_directories(MicroSDRUtil PRIVATE ${SDR_INCLUDE_DIR})

install(TARGETS MicroSDRUtil DESTINATION ${CMAKE_INSTALL_BINDIR})
add_executable(ANTSDRUtil
    ANTSDRUtil.cpp
)

target_include_directories(ANTSDRUtil PRIVATE ${SDR_INCLUDE_DIR})

install(TARGETS ANTSDRUtil DESTINATION ${CMAKE_INSTALL_BINDIR})
function(add_iqtaxi_example target source)
    add_executable(${target} ${source})
    target_link_libraries(${target} sdr_core sdr_driver)
    set_target_properties(${target} PROPERTIES
        BUILD_RPATH "$ORIGIN/../driver;$ORIGIN/../core"
    )
endfunction()

if(ENABLE_WIN)
    add_iqtaxi_example(e200_iq_example e200/e200_iq_example.cpp)
    add_iqtaxi_example(e200_dual_rx_capture e200/e200_dual_rx_capture.cpp)
    add_iqtaxi_example(e200_record_iq_test e200/e200_record_iq_test.cpp)
    add_iqtaxi_example(e200_replay_iq_test e200/e200_replay_iq_test.cpp)
    add_iqtaxi_example(e206_iq_example e206/e206_iq_example.cpp)
    add_iqtaxi_example(e206_record_iq_test e206/e206_record_iq_test.cpp)
    return()
endif()

add_iqtaxi_example(e100_test e100/e100_test.cpp)
add_iqtaxi_example(e100_full_duplex_test e100/e100_full_duplex_test.cpp)
add_iqtaxi_example(e100_record_iq_test e100/e100_record_iq_test.cpp)
add_iqtaxi_example(e100_replay_iq_test e100/e100_replay_iq_test.cpp)
add_iqtaxi_example(e100_control_stress_test e100/e100_control_stress_test.cpp)
add_iqtaxi_example(e100_rate_switch_probe e100/e100_rate_switch_probe.cpp)

add_iqtaxi_example(e200_iq_example e200/e200_iq_example.cpp)
add_iqtaxi_example(e200_dual_rx_capture e200/e200_dual_rx_capture.cpp)
add_iqtaxi_example(e200_record_iq_test e200/e200_record_iq_test.cpp)
add_iqtaxi_example(e200_replay_iq_test e200/e200_replay_iq_test.cpp)

add_iqtaxi_example(e206_iq_example e206/e206_iq_example.cpp)
add_iqtaxi_example(e206_record_iq_test e206/e206_record_iq_test.cpp)
add_iqtaxi_example(e206_replay_iq_test e206/e206_replay_iq_test.cpp)

add_iqtaxi_example(m300_xdma_smoke m300/m300_xdma_smoke.cpp)
add_iqtaxi_example(m300_discovery_probe m300/m300_discovery_probe.cpp)
add_iqtaxi_example(m300_discovery_protocol_test m300/m300_discovery_protocol_test.cpp)
add_iqtaxi_example(m300_iqtaxi_rx_example m300/m300_iqtaxi_rx_example.cpp)
add_iqtaxi_example(m300_iqtaxi_tx_example m300/m300_iqtaxi_tx_example.cpp)
add_iqtaxi_example(m300_iqtaxi_2r2t_test m300/m300_iqtaxi_2r2t_test.cpp)
add_iqtaxi_example(m300_rx_iq_probe m300/m300_rx_iq_probe.cpp)
add_iqtaxi_example(m300_rx_stream_probe m300/m300_rx_stream_probe.cpp)
add_iqtaxi_example(m300_tx_iq_probe m300/m300_tx_iq_probe.cpp)
add_iqtaxi_example(m300_ad9361_spi_probe m300/m300_ad9361_spi_probe.cpp)
add_iqtaxi_example(m300_ad9361_init_probe m300/m300_ad9361_init_probe.cpp)
add_iqtaxi_example(m300_ad9361_status_probe m300/m300_ad9361_status_probe.cpp)
add_iqtaxi_example(m300_flash_update m300/m300_flash_update.cpp)
target_link_options(m300_flash_update PRIVATE -Wl,--disable-new-dtags)

find_package(OpenGL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLFW3 REQUIRED glfw3)
pkg_check_modules(GLEW REQUIRED glew)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)
set(IQTAXI_IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

add_executable(e100_gui
    e100/e100_gui.cpp
    ${IQTAXI_IMGUI_SOURCES}
)

target_include_directories(e100_gui PRIVATE
    ${SDR_INCLUDE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${GLFW3_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

target_link_directories(e100_gui PRIVATE
    ${GLFW3_LIBRARY_DIRS}
    ${GLEW_LIBRARY_DIRS}
)

target_link_libraries(e100_gui PRIVATE
    sdr_core
    sdr_driver
    OpenGL::GL
    ${GLFW3_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

get_filename_component(E100_IMAGE_BUILDER_ROOT
    "${CMAKE_SOURCE_DIR}/../.."
    ABSOLUTE
)

target_compile_definitions(e100_gui PRIVATE
    IMGUI_IMPL_OPENGL_LOADER_GLEW
    E100_IMAGE_BUILDER_ROOT="${E100_IMAGE_BUILDER_ROOT}"
)
target_compile_options(e100_gui PRIVATE ${GLFW3_CFLAGS_OTHER} ${GLEW_CFLAGS_OTHER})
set_target_properties(e100_gui PROPERTIES
    BUILD_RPATH "$ORIGIN/../driver;$ORIGIN/../core"
)

add_executable(m300_iq_gui
    m300/m300_iq_gui.cpp
    ${IQTAXI_IMGUI_SOURCES}
)

target_include_directories(m300_iq_gui PRIVATE
    ${SDR_INCLUDE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${GLFW3_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

target_link_directories(m300_iq_gui PRIVATE
    ${GLFW3_LIBRARY_DIRS}
    ${GLEW_LIBRARY_DIRS}
)

target_link_libraries(m300_iq_gui PRIVATE
    sdr_core
    sdr_driver
    OpenGL::GL
    ${GLFW3_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${CMAKE_DL_LIBS}
    pthread
)

target_compile_definitions(m300_iq_gui PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLEW)
target_compile_options(m300_iq_gui PRIVATE ${GLFW3_CFLAGS_OTHER} ${GLEW_CFLAGS_OTHER})
target_link_options(m300_iq_gui PRIVATE -Wl,--disable-new-dtags -Wl,--no-as-needed)
set_target_properties(m300_iq_gui PROPERTIES
    BUILD_RPATH "$ORIGIN/../driver;$ORIGIN/../core"
)

add_executable(iqtaxi_firmware_update_gui
    tools/iqtaxi_firmware_update_gui.cpp
    ${IQTAXI_IMGUI_SOURCES}
)

target_include_directories(iqtaxi_firmware_update_gui PRIVATE
    ${SDR_INCLUDE_DIR}
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${GLFW3_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
)

target_link_directories(iqtaxi_firmware_update_gui PRIVATE
    ${GLFW3_LIBRARY_DIRS}
    ${GLEW_LIBRARY_DIRS}
)

target_link_libraries(iqtaxi_firmware_update_gui PRIVATE
    OpenGL::GL
    ${GLFW3_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${CMAKE_DL_LIBS}
    pthread
)

target_compile_definitions(iqtaxi_firmware_update_gui PRIVATE IMGUI_IMPL_OPENGL_LOADER_GLEW)
target_compile_options(iqtaxi_firmware_update_gui PRIVATE ${GLFW3_CFLAGS_OTHER} ${GLEW_CFLAGS_OTHER})

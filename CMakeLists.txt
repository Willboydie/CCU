cmake_minimum_required(VERSION 3.16)
project(lidar_scanning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT DEFINED BOOST_ROOT AND DEFINED ENV{BOOST_ROOT})
    set(BOOST_ROOT "$ENV{BOOST_ROOT}")
endif()

find_package(Boost REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
# Pi OS exposes Wayland without wayland-scanner. X11 is enough for the GUI tools.
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8
    GIT_SHALLOW TRUE
)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL)

set(LIDAR_CORE_SOURCES
    lib/rx/serial/SerialDevice.cpp
    lib/rx/parser/DeviceParser.cpp
    lib/rx/msg_queue/MessageQueue.cpp
    lib/rx/MessageConsumer.cpp
    lib/tx/LidarController.cpp
)

function(lidar_configure_target tgt)
    target_include_directories(${tgt} PRIVATE
        ${CMAKE_SOURCE_DIR}/lib
        ${Boost_INCLUDE_DIRS}
    )
    target_link_libraries(${tgt} PRIVATE Threads::Threads)
    if(TARGET Boost::headers)
        target_link_libraries(${tgt} PRIVATE Boost::headers)
    elseif(TARGET Boost::boost)
        target_link_libraries(${tgt} PRIVATE Boost::boost)
    endif()
    if(WIN32)
        target_compile_definitions(${tgt} PRIVATE
            _WIN32_WINNT=0x0A00
            NOMINMAX
            WIN32_LEAN_AND_MEAN
        )
        target_link_libraries(${tgt} PRIVATE ws2_32 mswsock)
    endif()
endfunction()

add_executable(lidar_serial_pipeline
    tests/lidar_serial_pipeline.cpp
    ${LIDAR_CORE_SOURCES}
)
lidar_configure_target(lidar_serial_pipeline)

add_executable(protocol_test
    tests/protocol_test.cpp
    lib/rx/parser/DeviceParser.cpp
)
lidar_configure_target(protocol_test)

add_executable(debug_message_test
    tests/debug_message_test.cpp
    ${LIDAR_CORE_SOURCES}
)
lidar_configure_target(debug_message_test)

add_executable(lidar_transceiver
    tests/lidar_transceiver.cpp
    lib/ui/TransceiverApplet.cpp
    ${LIDAR_CORE_SOURCES}
)
lidar_configure_target(lidar_transceiver)
target_link_libraries(lidar_transceiver PRIVATE imgui)

add_executable(scan_path_sim
    tests/scan_path_sim.cpp
    lib/ui/ScanPathApplet.cpp
    lib/scan/RasterInfinityPath.cpp
)
lidar_configure_target(scan_path_sim)
target_link_libraries(scan_path_sim PRIVATE imgui)

add_executable(scan_path_test
    tests/scan_path_test.cpp
    lib/scan/RasterInfinityPath.cpp
)
lidar_configure_target(scan_path_test)

add_executable(azimuth_finder_test
    tests/azimuth_finder_test.cpp
    lib/scan/AzimuthPointFinder.cpp
    lib/scan/PointCloud.cpp
    lib/scan/DwellPointCloud.cpp
)
lidar_configure_target(azimuth_finder_test)

add_executable(azimuth_finder_gui
    tests/azimuth_finder_gui.cpp
    lib/ui/AzimuthFinderApplet.cpp
    lib/scan/AzimuthPointFinder.cpp
    lib/scan/PointCloud.cpp
    lib/scan/DwellPointCloud.cpp
    ${LIDAR_CORE_SOURCES}
)
lidar_configure_target(azimuth_finder_gui)
target_link_libraries(azimuth_finder_gui PRIVATE imgui)

set(PROJECTOR_CORE_SOURCES
    lib/rx/serial/SerialDevice.cpp
    lib/rx/parser/DeviceParser.cpp
    lib/rx/parser/ProjectorParser.cpp
    lib/rx/msg_queue/MessageQueue.cpp
    lib/rx/MessageConsumer.cpp
    lib/tx/ProjectorController.cpp
)

add_executable(projector_protocol_test
    tests/projector_protocol_test.cpp
    lib/rx/parser/ProjectorParser.cpp
)
lidar_configure_target(projector_protocol_test)

add_executable(projector_point_gui
    tests/projector_point_gui.cpp
    lib/ui/ProjectorApplet.cpp
    ${PROJECTOR_CORE_SOURCES}
)
lidar_configure_target(projector_point_gui)
target_link_libraries(projector_point_gui PRIVATE imgui)

add_executable(projector_gui
    tests/projector_gui.cpp
    lib/ui/ProjectorApplet.cpp
    ${PROJECTOR_CORE_SOURCES}
)
lidar_configure_target(projector_gui)
target_link_libraries(projector_gui PRIVATE imgui)

add_executable(projector_calibration_gui
    tests/projector_calibration_gui.cpp
    lib/ui/ProjectorCalibrationApplet.cpp
    ${PROJECTOR_CORE_SOURCES}
)
lidar_configure_target(projector_calibration_gui)
target_link_libraries(projector_calibration_gui PRIVATE imgui)

add_executable(projector_esp_gui
    tests/projector_esp_gui.cpp
    lib/ui/ProjectorEspApplet.cpp
    lib/rx/serial/SerialTextPort.cpp
    ${PROJECTOR_CORE_SOURCES}
)
lidar_configure_target(projector_esp_gui)
target_link_libraries(projector_esp_gui PRIVATE imgui)

add_executable(promo_project_gui
    tests/promo_project_gui.cpp
    lib/ui/PromoProjectApplet.cpp
    ccu/path_engine.cpp
    ccu/galvo_model.cpp
    ccu/device_config.cpp
    ccu/bearing.cpp
    ccu/yaw.cpp
    ${PROJECTOR_CORE_SOURCES}
)
lidar_configure_target(promo_project_gui)
target_include_directories(promo_project_gui PRIVATE ${CMAKE_SOURCE_DIR}/ccu)
target_link_libraries(promo_project_gui PRIVATE imgui)
if(WIN32)
    target_link_libraries(promo_project_gui PRIVATE comdlg32)
endif()

add_executable(frontend_gui
    tests/frontend_gui.cpp
    lib/ui/FrontendApplet.cpp
    lib/tx/FrontendController.cpp
    lib/rx/serial/SerialDevice.cpp
    lib/rx/parser/DeviceParser.cpp
    lib/rx/msg_queue/MessageQueue.cpp
    lib/rx/MessageConsumer.cpp
)
lidar_configure_target(frontend_gui)
target_link_libraries(frontend_gui PRIVATE imgui)

function(ccu_configure_target tgt)
    target_include_directories(${tgt} PRIVATE
        ${CMAKE_SOURCE_DIR}/ccu
        ${CMAKE_SOURCE_DIR}/lib
    )
    target_compile_features(${tgt} PRIVATE cxx_std_20)
    if(MSVC)
        target_compile_options(${tgt} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${tgt} PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if(WIN32)
        target_compile_definitions(${tgt} PRIVATE
            NOMINMAX
            WIN32_LEAN_AND_MEAN
        )
    endif()
endfunction()

add_executable(ccu_protocol_test
    tests/ccu_protocol_test.cpp
    ccu/protocol.cpp
)
ccu_configure_target(ccu_protocol_test)

add_executable(ccu_yaw_test
    tests/ccu_yaw_test.cpp
    ccu/yaw.cpp
)
ccu_configure_target(ccu_yaw_test)

add_executable(ccu_command_allowed_test
    tests/ccu_command_allowed_test.cpp
    ccu/command_allowed.cpp
)
ccu_configure_target(ccu_command_allowed_test)

add_executable(ccu_galvo_model_test
    tests/ccu_galvo_model_test.cpp
    ccu/galvo_model.cpp
    ccu/device_config.cpp
)
ccu_configure_target(ccu_galvo_model_test)

add_executable(ccu_lidar_model_test
    tests/ccu_lidar_model_test.cpp
    ccu/lidar_model.cpp
    ccu/device_config.cpp
)
ccu_configure_target(ccu_lidar_model_test)

add_executable(ccu_path_engine_test
    tests/ccu_path_engine_test.cpp
    ccu/path_engine.cpp
    ccu/galvo_model.cpp
    ccu/device_config.cpp
)
ccu_configure_target(ccu_path_engine_test)

add_executable(ccu_bearing_test
    tests/ccu_bearing_test.cpp
    ccu/bearing.cpp
    ccu/yaw.cpp
    ccu/device_config.cpp
)
ccu_configure_target(ccu_bearing_test)

set(CCU_SESSION_SOURCES
    ccu/session.cpp
    ccu/device_config.cpp
    ccu/command_allowed.cpp
    ccu/protocol.cpp
    ccu/yaw.cpp
    ccu/bearing.cpp
    ccu/survey.cpp
    ccu/lidar.cpp
    ccu/lidar_model.cpp
    ccu/galvo_model.cpp
    ccu/project.cpp
    ccu/path_engine.cpp
    ccu/projector_tx.cpp
)

add_executable(ccu_session_test
    tests/ccu_session_test.cpp
    ${CCU_SESSION_SOURCES}
)
ccu_configure_target(ccu_session_test)

add_executable(ccu_telemetry_test
    tests/ccu_telemetry_test.cpp
    ccu/telemetry.cpp
    ${CCU_SESSION_SOURCES}
)
ccu_configure_target(ccu_telemetry_test)

add_executable(ccu_survey_test
    tests/ccu_survey_test.cpp
    ${CCU_SESSION_SOURCES}
)
ccu_configure_target(ccu_survey_test)

add_executable(ccu_project_test
    tests/ccu_project_test.cpp
    ${CCU_SESSION_SOURCES}
)
ccu_configure_target(ccu_project_test)

# H2.0. Localhost raw bytes. The tablet test is the other program.
add_executable(ccu_h2_0
    tests/ccu_h2_0_bytes.cpp
)
ccu_configure_target(ccu_h2_0)
if(WIN32)
    target_link_libraries(ccu_h2_0 PRIVATE ws2_32)
endif()

# H2.1. Localhost DEVICE_ID. The tablet parser is the other program.
add_executable(ccu_h2_1
    tests/ccu_h2_1_device_id.cpp
    ccu/protocol.cpp
)
ccu_configure_target(ccu_h2_1)
if(WIN32)
    target_link_libraries(ccu_h2_1 PRIVATE ws2_32)
endif()

add_executable(ccu_h2_2
    tests/ccu_h2_2_dongle_test.cpp
    ccu/dongle_link.cpp
    ${CCU_SESSION_SOURCES}
)
ccu_configure_target(ccu_h2_2)

add_executable(ccu_h2_4
    tests/ccu_h2_4_idle.cpp
    ccu/session_poll.cpp
    ccu/dongle_link.cpp
    ${CCU_SESSION_SOURCES}
)
ccu_configure_target(ccu_h2_4)
if(WIN32)
    target_link_libraries(ccu_h2_4 PRIVATE ws2_32)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(ccu
        ccu/main.cpp
        ccu/session_poll.cpp
        ccu/serial.cpp
        ccu/dongle.cpp
        ccu/dongle_link.cpp
        ccu/angle.cpp
        ccu/projector.cpp
        ccu/lidar_port.cpp
        ccu/telemetry.cpp
        ${CCU_SESSION_SOURCES}
    )
    ccu_configure_target(ccu)
    target_link_libraries(ccu PRIVATE Threads::Threads)

    add_executable(ccu_h2
        tests/ccu_h2_bench.cpp
        ccu/serial.cpp
        ccu/angle.cpp
        ${CCU_SESSION_SOURCES}
    )
    ccu_configure_target(ccu_h2)

    add_executable(ccu_lidar_link
        tests/lidar_link_test.cpp
        ccu/serial.cpp
    )
    ccu_configure_target(ccu_lidar_link)

    add_executable(ccu_byte_swap
        tests/byte_swap_test.cpp
        ccu/serial.cpp
    )
    ccu_configure_target(ccu_byte_swap)
endif()

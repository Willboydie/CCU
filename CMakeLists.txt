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

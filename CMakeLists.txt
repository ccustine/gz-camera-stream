cmake_minimum_required(VERSION 3.16)
project(gz-camera-stream VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Gazebo packages
find_package(gz-sim REQUIRED COMPONENTS rendering)
find_package(gz-rendering REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)

# Find FFmpeg
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
    libavcodec libavformat libswscale libavutil)

# Build the plugin as a shared library
add_library(gz-sim-camera-stream-system SHARED
    src/CameraStream.cc
    src/StreamContext.cc
)

target_include_directories(gz-sim-camera-stream-system PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(gz-sim-camera-stream-system
    PUBLIC
        gz-sim::gz-sim
        gz-sim::rendering
        gz-rendering::gz-rendering
        gz-transport::gz-transport
        gz-msgs::gz-msgs
        gz-common::gz-common
    PRIVATE
        gz-plugin::register
        PkgConfig::FFMPEG
)

# Install to the gz-sim plugin directory so Gazebo can find it
# GZ_SIM_PLUGIN_INSTALL_DIR is typically lib/
install(TARGETS gz-sim-camera-stream-system
    LIBRARY DESTINATION lib
)

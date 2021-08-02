cmake_minimum_required(VERSION 3.0)
project(location_monitor_rust)

find_package(catkin REQUIRED)
catkin_package()

add_custom_target(location_monitor_rust
   ALL
   COMMAND cargo build --release -p location_monitor_rust
   COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_BINARY_DIR}/cargo/release/location_monitor_rust ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/location_monitor_rust
   COMMENT "Building location_monitor_rust!!!"
)
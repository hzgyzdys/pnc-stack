cmake_minimum_required(VERSION 3.16)
project(pnc_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pnc_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Both nodes are built as components (loadable into a container) and as
# standalone executables via rclcpp_components_register_node.
add_library(pnc_control_components SHARED
  src/tracking_controller_node.cpp
  src/reference_path_node.cpp
)
target_link_libraries(pnc_control_components
  pnc_core::pnc_core
  rclcpp::rclcpp
  rclcpp_components::component
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

rclcpp_components_register_node(pnc_control_components
  PLUGIN "pnc_control::TrackingControllerNode"
  EXECUTABLE tracking_controller
)
rclcpp_components_register_node(pnc_control_components
  PLUGIN "pnc_control::ReferencePathNode"
  EXECUTABLE reference_path
)

install(TARGETS pnc_control_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()

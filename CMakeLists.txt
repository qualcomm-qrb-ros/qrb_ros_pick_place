cmake_minimum_required(VERSION 3.8)
project(qrb_ros_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs
)

add_library(pick_place_node_lib SHARED
  src/pick_place_node.cpp
)

target_include_directories(pick_place_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pick_place_node_lib
  rclcpp
  rclcpp_action
  geometry_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_core
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pick_place_node_lib "${cpp_typesupport_target}")

add_executable(pick_place_node src/main.cpp)
target_link_libraries(pick_place_node pick_place_node_lib)
target_include_directories(pick_place_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS pick_place_node_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS pick_place_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(pick_place_node_lib)
ament_export_targets(export_${PROJECT_NAME})

ament_package()

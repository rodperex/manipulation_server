cmake_minimum_required(VERSION 3.5)
project(manipulation_action_server)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(manipulation_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
# find_package(backward_ros REQUIRED)


set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    rclcpp_components
    ament_index_cpp
    moveit_task_constructor_core
    manipulation_interfaces
    # backward_ros
    rclcpp_cascade_lifecycle
)

include_directories(
include
)
add_library(manipulation_server SHARED
  src/manipulation/ManipulationServer.cpp 
  # src/manipulation/MTCNode.cpp
  src/manipulation/manipulation_behaviors.cpp
)
ament_target_dependencies(manipulation_server ${dependencies})

add_executable(server
  src/main_srv.cpp
)
ament_target_dependencies(server
  ${dependencies}
)
target_link_libraries(server manipulation_server)

add_executable(client
  src/main_cli.cpp
  src/manipulation/MoveToActionClient.cpp
)
ament_target_dependencies(client
  ${dependencies}
)

rclcpp_components_register_nodes(manipulation_server
  "manipulation::ManipulationServer"
)

install(TARGETS
  server
  client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(${PROJECT_NAME})
ament_package()

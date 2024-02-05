cmake_minimum_required(VERSION 3.5)
project(action_server)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(manipulation_action REQUIRED)


set(dependencies
    rclcpp
    rclcpp_action
    ament_index_cpp
    moveit_task_constructor_core
    manipulation_action
)

include_directories(
include
)

add_executable(server
  src/main_srv.cpp
  src/manipulation/MoveToActionServer.cpp 
  src/manipulation/MTCNode.cpp
)
ament_target_dependencies(server
  ${dependencies}
)

add_executable(client
  src/main_cli.cpp
  src/manipulation/MoveToActionClient.cpp
)
ament_target_dependencies(client
  ${dependencies}
)

install(TARGETS
  server
  # client
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

cmake_minimum_required(VERSION 3.8)
project(manipulation_interfaces)

find_package(ament_cmake REQUIRED)
# find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(action_msgs
    action/MoveToPredefined.action
)

# set(srv_files
#   srv/Srv1.srv
#   srv/Srv2.srv
# )

rosidl_generate_interfaces(${PROJECT_NAME}
    ${action_msgs} #${srv_files}
    #DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_package()

cmake_minimum_required(VERSION 3.16)
project(jaetrobi_controller_msgs)

# using this instead of visibility macros
# S1 from https://github.com/ros-controls/ros2_controllers/issues/1053
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowCartesianTrajectory.action"
  DEPENDENCIES
    geometry_msgs
    moveit_msgs
)

ament_package()
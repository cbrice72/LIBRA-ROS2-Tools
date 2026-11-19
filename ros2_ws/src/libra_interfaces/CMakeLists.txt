################################################################################
# @file   CMakeLists.txt
# @brief  Message definitions for LIBRA App's topics; CMakeLists file.
#
# @author Christian Brice
################################################################################

cmake_minimum_required(VERSION 3.8)
project(libra_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate message types
rosidl_generate_interfaces(
    ${PROJECT_NAME}
    msg/CwState.msg
    msg/Diagnostics.msg
    DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

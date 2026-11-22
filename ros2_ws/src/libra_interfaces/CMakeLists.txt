################################################################################
# @file   CMakeLists.txt
# @brief  Message and service definitions for LIBRA; CMakeLists file.
#
# @author Christian Brice
################################################################################

cmake_minimum_required(VERSION 3.8)
project(libra_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Generate message and service types
rosidl_generate_interfaces(
    ${PROJECT_NAME}
    msg/Counterweight.msg
    msg/CwState.msg
    msg/Diagnostics.msg
    srv/GetTipReach.srv
    DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

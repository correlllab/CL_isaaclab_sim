cmake_minimum_required(VERSION 3.18)
project(correll-ros2-imu LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PYTHONVERSION "3.11")
set(PYTHON_EXECUTABLE "/isaac-sim/kit/python/bin/python3.11")

find_package(pybind11 CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(PYBIND11_PYTHON_VERSION "3.11")


file(GLOB_RECURSE ROS2_HEADERS /opt/ros/jazzy/include//*.h*)

set(CORRELL_COMMON_INCLUDE
  /workspace/golem/isaac/mateo_ws/CL_isaaclab_sim/src/cpp/correll-common/include
)

add_library(${PROJECT_NAME} SHARED
  src/Ros2ImuNode.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC   
    include
    ${CORRELL_COMMON_INCLUDE}
    ${Python3_INCLUDE_DIRS}
    /opt/ros/jazzy/include/rclcpp
    /opt/ros/jazzy/include/rcl_interfaces
    /opt/ros/jazzy/include/rosidl_runtime_cpp
    /opt/ros/jazzy/include/rosidl_runtime_c
    /opt/ros/jazzy/include/service_msgs
    /opt/ros/jazzy/include/builtin_interfaces
    /opt/ros/jazzy/include/rosidl_typesupport_interface
    /opt/ros/jazzy/include/rcutils
    /opt/ros/jazzy/include/rcl
    /opt/ros/jazzy/include/rmw
    /opt/ros/jazzy/include/rcpputils
    /opt/ros/jazzy/include/rcl_yaml_param_parser
    /opt/ros/jazzy/include/type_description_interfaces
    /opt/ros/jazzy/include/rosidl_dynamic_typesupport
    /opt/ros/jazzy/include/tracetools
    /opt/ros/jazzy/include/libstatistics_collector
    /opt/ros/jazzy/include/statistics_msgs
    /opt/ros/jazzy/include/rosidl_typesupport_introspection_cpp
    /opt/ros/jazzy/include/sensor_msgs
    /opt/ros/jazzy/include/std_msgs
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    pybind11::module
    sensor_msgs::sensor_msgs
    std_msgs::std_msgs
    rclcpp::rclcpp
)


pybind11_add_module(ros2_imu_node_py
  src/Ros2ImuNode.cpp
)

target_link_libraries(ros2_imu_node_py PRIVATE
    correll-ros2-imu
)


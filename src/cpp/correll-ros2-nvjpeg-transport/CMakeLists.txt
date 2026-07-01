#i cant even lie its like miraculous this cmake works
cmake_minimum_required(VERSION 3.18)
project(correll-ros2-nvjpeg-transport LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PYTHONVERSION "3.11")
set(PYTHON_EXECUTABLE "/isaac-sim/kit/python/bin/python3.11")

find_package(pybind11 CONFIG REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ISAAC_SIM_ROOT /isaac-sim)
set(PYBIND11_PYTHON_VERSION "3.11")

set(NVJPEG_LIB
  ${ISAAC_SIM_ROOT}/extscache/omni.sensors.nv.camera-1.0.0+69cbf6ad.lx64.r/bin/libnvjpeg.so
)

file(GLOB_RECURSE ROS2_HEADERS /opt/ros/jazzy/include//*.h*)

set(CUDA_INCLUDE
  ${ISAAC_SIM_ROOT}/kit/python/lib/python3.11/site-packages/nvidia/nvjpeg/include
)

set(CORRELL_COMMON_INCLUDE
  /workspace/hams/isaac/mateo_ws/CL_isaaclab_sim/src/cpp/correll-common/include
)

add_library(${PROJECT_NAME} SHARED
  src/NvjpegEncoder.cpp
  src/Ros2NvjpegTransport.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC   
    include
    ${CORRELL_COMMON_INCLUDE}
    ${Python3_INCLUDE_DIRS}
    ${CUDA_INCLUDE}
    /isaac-sim/kit/python/lib/python3.11/site-packages/nvidia/cu13/include
    /isaac-sim/kit/python/lib/python3.11/site-packages/triton/backends/nvidia/include
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
    ${NVJPEG_LIB}
    /isaac-sim/kit/python/lib/python3.11/site-packages/nvidia/cu13/lib/libcudart.so.13
    sensor_msgs::sensor_msgs
    std_msgs::std_msgs
    rclcpp::rclcpp
)


#for debugging, omitted just to be lightweight
pybind11_add_module(nvjpeg_encoder_py
  src/NvjpegEncoder.cpp
)

target_link_libraries(nvjpeg_encoder_py PRIVATE
    correll-ros2-nvjpeg-transport
)

pybind11_add_module(ros2_nvjpeg_compressed_image_publisher_py
    src/Ros2NvjpegTransport.cpp
)

target_link_libraries(ros2_nvjpeg_compressed_image_publisher_py PRIVATE
    correll-ros2-nvjpeg-transport
)

#target_link_libraries(correll.ros2.nvjpeg.transport.dummy PRIVATE
#  correll.ros2.nvjpeg.transport
#)

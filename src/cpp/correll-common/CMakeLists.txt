cmake_minimum_required(VERSION 3.12)

project(correll-common LANGUAGES CXX)

add_library(correll-common INTERFACE)

target_compile_features(correll-common INTERFACE cxx_std_20)

target_include_directories(correll-common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

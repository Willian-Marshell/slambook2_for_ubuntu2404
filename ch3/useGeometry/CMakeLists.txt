cmake_minimum_required( VERSION 3.8 )
project( geometry )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

add_executable(eigenGeometry useGeometry.cpp)

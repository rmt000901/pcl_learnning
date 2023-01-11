cmake_minimum_required(VERSION 3.23)
set(CMAKE_CXX_STANDARD 14)
project(pcd_pinjie)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}
        /usr/local/include/yaml-cpp)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcd_pinjie concatenate_clouds.cpp concatenate_clouds.cpp)
target_link_libraries (pcd_pinjie ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries (pcd_pinjie /usr/local/lib/libyaml-cpp.a)
cmake_minimum_required(VERSION 3.20)
project (cylinder_detection)

find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(Open3D REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# For use_pcl setting, examples use:   depthai::all  &  depthai::opencv if off
add_executable(cylinder_detection main.cpp)
target_link_libraries(cylinder_detection PRIVATE depthai::core ${OpenCV_LIBS} Open3D::Open3D ${PCL_LIBRARIES})

set_property(TARGET cylinder_detection PROPERTY CXX_STANDARD 17)
set_property(TARGET cylinder_detection PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET cylinder_detection PROPERTY CXX_EXTENSIONS OFF)

option(BUILD_EXAMPLES "Build Examples" OFF)
if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# DEPTHAI_PCL_SUPPORT
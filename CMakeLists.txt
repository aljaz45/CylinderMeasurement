cmake_minimum_required(VERSION 3.20)
project (cylinder_detection)

find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(Open3D REQUIRED)

# For use_pcl setting, examples use:   depthai::all  &  depthai::opencv if off
add_executable(cylinder_detection main2.cpp)
target_link_libraries(cylinder_detection PRIVATE depthai::core ${OpenCV_LIBS})

set_property(TARGET cylinder_detection PROPERTY CXX_STANDARD 17)
set_property(TARGET cylinder_detection PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET cylinder_detection PROPERTY CXX_EXTENSIONS OFF)


# DEPTHAI_PCL_SUPPORT
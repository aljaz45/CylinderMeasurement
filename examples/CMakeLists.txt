cmake_minimum_required(VERSION 3.20)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depthai CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set_property(TARGET cylinder_detection PROPERTY CXX_STANDARD 17)
set_property(TARGET cylinder_detection PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET cylinder_detection PROPERTY CXX_EXTENSIONS OFF)


add_executable(get_cylinder_dimensions get_cylinder_dimensions.cpp)
target_link_libraries(get_cylinder_dimensions PRIVATE ${PCL_LIBRARIES})

add_executable(record_pointcloud record_pointcloud.cpp)
target_link_libraries(record_pointcloud PRIVATE depthai::core ${OpenCV_LIBS} ${PCL_LIBRARIES})

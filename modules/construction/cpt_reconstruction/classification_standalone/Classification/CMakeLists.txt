cmake_minimum_required(VERSION 3.19)
project(Classification)

set(CMAKE_CXX_STANDARD 14)

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(PCL 1.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(SCIP REQUIRED)
find_package(TBB REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(Classification main.cpp)
target_link_libraries(Classification CGAL::CGAL CGAL::CGAL_Core ${PCL_LIBRARIES}
        Eigen3::Eigen pcl_visualization /usr/include /usr/local/lib/libscip.so /usr/lib/x86_64-linux-gnu/libtbb.so)
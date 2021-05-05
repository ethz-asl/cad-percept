cmake_minimum_required(VERSION 3.17)
project(ReconstructionLP)

set(CMAKE_CXX_STANDARD 14)

find_package(CGAL REQUIRED COMPONENTS Core)
find_package(PCL 1.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(SCIP REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ReconstructionLP main.cpp)
target_link_libraries(ReconstructionLP CGAL::CGAL CGAL::CGAL_Core ${PCL_LIBRARIES}
        Eigen3::Eigen pcl_visualization /usr/include /usr/local/lib/libscip.so)
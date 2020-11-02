cmake_minimum_required(VERSION 3.0.2)
project(cpt_pointlaser_loc_ros)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME}
  src/kinova_localizer.cpp
  src/mabi_localizer.cpp
)

cs_add_executable(kinova_localizer_node src/kinova_localizer_node.cpp)

target_link_libraries(kinova_localizer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

cs_install()
cs_export()
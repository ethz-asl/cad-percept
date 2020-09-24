cmake_minimum_required(VERSION 3.0.2)
project(cpt_pointlaser_loc_ros)

add_definitions(-std=c++14)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(${PROJECT_NAME}_node src/cpt_pointlaser_loc_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

cs_install()
cs_export()
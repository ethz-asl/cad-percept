cmake_minimum_required(VERSION 2.8.3)
project(cgal_definitions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/mesh_model.cc
)


############
# BINARIES #
############

############
#  TESTS   #
############
catkin_add_gtest(${PROJECT_NAME}_tests
    test/test_main.cpp
    test/test_mesh_model.cpp)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()

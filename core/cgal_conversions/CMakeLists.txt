cmake_minimum_required(VERSION 2.8.3)
project(cgal_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/mesh_conversions.cc
        src/eigen_conversions.cc
        src/tf_conversions.cc
        )

############
# BINARIES #
############

############
#  TESTS   #
############
catkin_add_gtest(${PROJECT_NAME}_tests
        test/test_main.cpp
        test/test_mesh_messages.cpp
        test/test_eigen_conversions.cpp
        test/test_cgal_eigen_adapter_2d.cpp
        test/test_cgal_eigen_adapter_3d.cpp)

target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

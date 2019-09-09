cmake_minimum_required(VERSION 2.8.3)
project(cpt_utils)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/cpt_utils.cc
        src/perf.cc
        )


############
# BINARIES #
############
cs_add_executable(mesh_publisher_node
        src/mesh_publisher.cc
        src/mesh_publisher_node.cc
        )

############
# Test #
############

catkin_add_gtest(perf_overhead_test test/perf_overhead_test.cc)
target_link_libraries(perf_overhead_test ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()

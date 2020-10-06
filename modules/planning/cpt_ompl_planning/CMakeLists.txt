cmake_minimum_required(VERSION 2.8.3)
project(cpt_ompl_planning)
add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})
add_definitions(-std=c++17 -Wall -Wextra)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/ompl_mesh_sampling_planner.cpp
        src/ompl_mesh_projecting_planner.cpp)

target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES}
        cgal_definitions)


cs_add_executable(evaluation_node
        src/evaluation_node.cpp)
target_link_libraries(evaluation_node
        ${PROJECT_NAME}
        ${OMPL_LIBRARIES}
        cgal_definitions)



cs_add_executable(tuning_node
        src/tuning_node.cpp)

target_link_libraries(tuning_node
        ${PROJECT_NAME}
        ${OMPL_LIBRARIES}
        cgal_definitions)

cs_add_executable(kpi_test_node
        src/kpi_test.cpp)

target_link_libraries(kpi_test_node
        ${PROJECT_NAME}
        ${OMPL_LIBRARIES}
        cgal_definitions)
##########
# EXPORT #
##########
cs_install()
cs_export()

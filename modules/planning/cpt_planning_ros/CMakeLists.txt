cmake_minimum_required(VERSION 2.8.3)
project(cpt_planning_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(CUDAToolkit)
find_library(NVINFER  NAMES nvinfer REQUIRED)
find_library(NVINFERPLUGIN NAMES nvinfer_plugin)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSER NAMES nvonnxparser)
find_library(NVONNXPARSERRUNTIME NAMES nvonnxparser_runtime)

add_definitions(-std=c++17 -Wall -Wextra -fopenmp)


#############
# Executables #
#############

cs_add_executable(omav_planner_node
        src/omav_planner.cc
        src/omav_planner_node.cc
        )

cs_add_executable(omav_planner_tof
        src/omav_planner_tof.cc
                )



#cs_add_executable(tensorrt

        #src/tensorrt_test.cc
        #)

#target_link_libraries(tensorrt CUDA::cudart nvinfer)

############
#  TESTS   #
############


##########
# EXPORT #
##########
cs_install()
cs_export()

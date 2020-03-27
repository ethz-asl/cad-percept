cmake_minimum_required(VERSION 2.8.3)
project(cpt_matching_algorithms)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

###########
## LIBRARIES ##
###########
cs_add_library(test_matcher
     src/test_matcher/test_matcher.cpp
     src/test_matcher/matcher/template_matcher.cpp
     src/test_matcher/matcher/go_icp_matcher.cpp
	)


#############
## BINARIES ##
#############
cs_add_executable(test_matcher_node
    src/test_matcher/test_matcher_node.cpp
    )
cs_add_executable(data_reader
    src/data_creator/data_reader.cpp
    )
cs_add_executable(sim_lidar_node
    src/test_matcher/sim_lidar_node.cpp
    )


target_link_libraries(test_matcher_node test_matcher)


#############
## Testing ##
#############

#############
## EXPORT ##
#############
cs_install()
cs_export()

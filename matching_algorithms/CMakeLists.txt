cmake_minimum_required(VERSION 2.8.3)
project(matching_algorithms)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

###########
## LIBRARIES ##
###########
cs_add_library(test_Matcher
	 src/test_Matcher/test_Matcher.cpp
     src/test_Matcher/matcher/template_matcher.cpp
     src/test_Matcher/matcher/go_icp_matcher.cpp
	)


#############
## BINARIES ##
#############
cs_add_executable(test_Matcher_node
    src/test_Matcher/test_Matcher_node.cpp
    )
cs_add_executable(dataReader
    src/data_creator/dataReader.cpp
    )
cs_add_executable(simlidar_node
    src/test_Matcher/simlidar_node.cpp
    )


target_link_libraries(test_Matcher_node test_Matcher)


#############
## Testing ##
#############

#############
## EXPORT ##
#############
cs_install()
cs_export()

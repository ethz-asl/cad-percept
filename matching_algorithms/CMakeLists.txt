cmake_minimum_required(VERSION 2.8.3)
project(matching_algorithms)

find_package(catkin_simple REQUIRED)
find_package(PCL 1.7 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

###########
## LIBRARIES ##
###########
cs_add_library(test_Matcher
	src/test_Matcher/test_Matcher.cpp
	src/test_Matcher/feature_matcher.cpp
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


target_link_libraries(test_Matcher_node test_Matcher ${PCL_LIBRARIES})
target_link_libraries(dataReader ${catkin_LIBRARIES})


#############
## Testing ##
#############

#############
## EXPORT ##
#############
cs_install()
cs_export()

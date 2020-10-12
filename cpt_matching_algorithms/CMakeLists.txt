cmake_minimum_required(VERSION 2.8.3)
project(cpt_matching_algorithms)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(PCL REQUIRED)

add_definitions(-std=c++11 -Wall -Wextra)

###########
## LIBRARIES ##
###########
cs_add_library(test_matcher
     src/test_matcher/test_matcher.cpp
     src/test_matcher/matcher/go_icp_matcher.cpp
     src/test_matcher/bounded_planes.cpp
     src/cloud_filter/cloud_filter.cpp
     src/plane_extraction/plane_extraction.cpp
     src/plane_extraction/rht_accumulator.cpp
     src/plane_matching/plane_matching.cpp
	)

cs_add_library(evaluation_runner_lib
     src/test_matcher/matcher/go_icp_matcher.cpp
     src/test_matcher/bounded_planes.cpp
     src/cloud_filter/cloud_filter.cpp
     src/plane_extraction/plane_extraction.cpp
     src/plane_extraction/rht_accumulator.cpp
     src/plane_matching/plane_matching.cpp
     src/test_matcher/evaluation_runner.cpp
     )

cs_add_library(evaluation_goicp_lib
     src/test_matcher/matcher/go_icp_matcher.cpp
     src/test_matcher/bounded_planes.cpp
     src/cloud_filter/cloud_filter.cpp
     src/plane_extraction/plane_extraction.cpp
     src/plane_extraction/rht_accumulator.cpp
     src/plane_matching/plane_matching.cpp
     src/test_matcher/evaluation_go_icp.cpp
     )

cs_add_library(descriptor_matcher
	src/test_matcher/matcher/3d_descriptor_matcher.cpp
	)
target_link_libraries(descriptor_matcher ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#############
## BINARIES ##
#############
cs_add_executable(test_matcher_node
    src/test_matcher/test_matcher_node.cpp
    )
cs_add_executable(data_reader
    src/data_creator/data_reader.cpp
    )
cs_add_executable(data_to_pcl_reader
    src/data_creator/data_to_pcl_reader.cpp
    )
cs_add_executable(sim_lidar_node
    src/test_matcher/sim_lidar_node.cpp
    )
cs_add_executable(evaluation_runner_node
    src/test_matcher/evaluation_runner.cpp
    )
cs_add_executable(evaluation_go_icp_node
    src/test_matcher/evaluation_go_icp.cpp
    )

target_link_libraries(test_matcher_node test_matcher  yaml-cpp)
target_link_libraries(evaluation_runner_node evaluation_runner_lib yaml-cpp descriptor_matcher)
target_link_libraries(evaluation_go_icp_node evaluation_goicp_lib yaml-cpp)

#############
## Testing ##
#############
find_package(rostest REQUIRED)
add_rostest_gtest(${PROJECT_NAME}_tests
    test/testlaunch.launch
    test/test_main.cpp
    test/test_plane_extraction.cpp)
target_link_libraries(${PROJECT_NAME}_tests test_matcher yaml-cpp)

#############
## EXPORT ##
#############
cs_install()
cs_export()

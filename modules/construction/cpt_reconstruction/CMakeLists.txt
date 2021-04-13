cmake_minimum_required(VERSION 2.8.3)
project(cpt_reconstruction)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)


add_definitions(-std=c++11 -Wall -Wextra)


#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/reconstruction_points_publisher.cpp
        src/reconstruction_points_subscriber.cpp
		src/reconstruction_preprocess_model.cpp
		src/reconstruction_mesh_generation.cpp
        )

############
# BINARIES #
############
cs_add_executable(reconstruction_points_publisher_node
        src/reconstruction_points_publisher_node.cpp
        )
target_link_libraries(reconstruction_points_publisher_node ${PROJECT_NAME})

cs_add_executable(reconstruction_points_subscriber_node
        src/reconstruction_points_subscriber_node.cpp
        )
target_link_libraries(reconstruction_points_subscriber_node ${PROJECT_NAME})

cs_add_executable(reconstruction_mesh_generation_node
		src/reconstruction_mesh_generation_node.cpp
		)
target_link_libraries(reconstruction_mesh_generation_node ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()

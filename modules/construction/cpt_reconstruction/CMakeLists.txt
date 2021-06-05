cmake_minimum_required(VERSION 3.1.0)
project(cpt_reconstruction)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14 -Wall -Wextra)

find_package(SCIP REQUIRED)
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})
#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/reconstruction_points_publisher.cpp
		src/reconstruction_shape_detection.cpp
        src/reconstruction_model.cpp
		src/reconstruction_clustering.cpp
		src/reconstruction_proposal_selection.cpp
		src/reconstruction_mesh_generation.cpp
		src/reconstruction_classification.cpp
        )

############
# BINARIES #
############

cs_add_executable(reconstruction_points_publisher_node
        src/reconstruction_points_publisher_node.cpp
        )
target_link_libraries(reconstruction_points_publisher_node ${PROJECT_NAME} /usr/include /usr/local/lib/libscip.so)

cs_add_executable(reconstruction_shape_detection_node
		src/reconstruction_shape_detection_node.cpp
        )
target_link_libraries(reconstruction_shape_detection_node ${PROJECT_NAME} /usr/include /usr/local/lib/libscip.so)

cs_add_executable(reconstruction_clustering_node
		src/reconstruction_clustering_node.cpp
		)
target_link_libraries(reconstruction_clustering_node ${PROJECT_NAME} /usr/include /usr/local/lib/libscip.so)

cs_add_executable(reconstruction_classification_node
		src/reconstruction_classification_node.cpp
		)
target_link_libraries(reconstruction_classification_node ${PROJECT_NAME} /usr/include /usr/local/lib/libscip.so)

cs_add_executable(reconstruction_mesh_generation_node
		src/reconstruction_mesh_generation_node.cpp
		)
target_link_libraries(reconstruction_mesh_generation_node ${PROJECT_NAME} /usr/include /usr/local/lib/libscip.so)

##########
# EXPORT #
##########
cs_install()

cs_install_scripts(src/reconstruction_model_integration_node.py)

cs_export()

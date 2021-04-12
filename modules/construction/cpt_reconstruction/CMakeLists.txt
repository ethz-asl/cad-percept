cmake_minimum_required(VERSION 3.10.2)
project(cpt_reconstruction)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

set(CMAKE_CXX_STANDARD 14)

add_message_files(DIRECTORY msg FILES coordinates.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()


find_package(CGAL REQUIRED COMPONENTS Core)
find_package(PCL 1.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
        src/reconstruction_points_publisher.cpp
        src/reconstruction_points_subscriber.cpp
	)

add_executable(publisher src/reconstruction_points_publisher_node.cpp)
target_link_libraries(publisher ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS} ${PCL_IO_LIBRARIES})

add_executable(subscriber src/reconstruction_points_subscriber_node.cpp)
target_link_libraries(subscriber ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARY_DIRS} ${PCL_IO_LIBRARIES})

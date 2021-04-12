#include <cpt_reconstruction/reconstruction_points_publisher.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

int main(int argc, char** argv){
    ros::init(argc, argv, "reconstruction_publisher_node");
    ros::NodeHandle nodeHandle;

    std::string path_scan ="/home/philipp/Schreibtisch/scan1.ply";
	
	cad_percept::cpt_reconstruction::ReconstructionPointsPublisher publisher(nodeHandle, path_scan, 1);
    publisher.publishPoints();

    return 0;
}
#include <cpt_reconstruction/reconstruction_points_subscriber.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string>

int main(int argc, char** argv){
    ros::init(argc, argv, "reconstruction_subscriber_node");
    ros::NodeHandle nodeHandle;

	cad_percept::cpt_reconstruction::ReconstructionPointsSubscriber subscriber(nodeHandle);

    return 0;
}
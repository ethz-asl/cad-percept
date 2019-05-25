#include <ros/ros.h>

#include "cpt_changes/changes_ros.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "changes_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	std::shared_ptr<cad_percept::changes::ChangesRos> changes_ros_node;

	// Hand over the handle to the object.
	changes_ros_node = std::make_shared<cad_percept::changes::ChangesRos>(nh, nh_private);

	ros::spin();

	return 0;
}

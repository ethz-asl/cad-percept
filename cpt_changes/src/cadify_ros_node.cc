#include <ros/ros.h>

#include "cpt_changes/cadify_ros.h"

int main(int argc, char **argv){
    ros::init(argc, argv, "cadify_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::shared_ptr<cad_percept::cadify::CadifyRos> cadify_node;

    // Hand over the handle to the object.
    cadify_node = std::make_shared<cad_percept::cadify::CadifyRos>(nh, nh_private);

    ros::spin();

    return 0;
}

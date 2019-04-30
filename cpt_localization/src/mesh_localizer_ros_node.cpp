#include <ros/ros.h>

#include "cpt_localization/mesh_localizer_ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "cadifyer_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::shared_ptr<cad_percept::localization::MeshLocalizerRos> mesh_localizer_node;

  // Hand over the handle to the object.
  mesh_localizer_node = std::make_shared<cad_percept::localization::MeshLocalizerRos>
      (nh,
      nh_private);

  ros::spin();

  return 0;
}

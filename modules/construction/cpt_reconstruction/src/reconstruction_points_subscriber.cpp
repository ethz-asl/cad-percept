#include <cpt_reconstruction/reconstruction_points_subscriber.h>

#include <string>
#include "cpt_reconstruction/coordinates.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace cad_percept {
namespace cpt_reconstruction {
ReconstructionPointsSubscriber::ReconstructionPointsSubscriber(ros::NodeHandle nodeHandle) {
  nodeHandle_ = nodeHandle;
  subscriber_ =
      nodeHandle_.subscribe("point", 1, &ReconstructionPointsSubscriber::messageCallback, this);
  ros::spin();
}

void ReconstructionPointsSubscriber::messageCallback(const ::cpt_reconstruction::coordinates& msg) {
  ROS_INFO("[Listener] I heard %f %f %f\n", msg.x, msg.y, msg.z);
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept
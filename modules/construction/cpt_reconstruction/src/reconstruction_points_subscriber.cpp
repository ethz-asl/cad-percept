#include <cpt_reconstruction/reconstruction_preprocess_model.h>
#include <cpt_reconstruction/reconstruction_points_subscriber.h>

#include <pcl/point_types.h>
#include <cmath>
#include <string>
#include "cpt_reconstruction/coordinates.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace cad_percept {
namespace cpt_reconstruction {
ReconstructionPointsSubscriber::ReconstructionPointsSubscriber(
    ros::NodeHandle nodeHandle, PreprocessModel* model)
    : nodeHandle_(nodeHandle), model_(model) {
  subscriber_ = nodeHandle_.subscribe(
      "point", 1, &ReconstructionPointsSubscriber::messageCallback, this);
  ros::spin();
}

void ReconstructionPointsSubscriber::messageCallback(
    const ::cpt_reconstruction::coordinates& msg) {
  pcl::PointXYZ p(msg.x, msg.y, msg.z);
  model_->queryTree(p);
  float min_dist = model_->getMinDistance();
  if (std::abs(min_dist) >= 0.015) {
    model_->addOutlier(msg.idx, p);
    ROS_INFO("[Subscriber] Received %f %f %f with min_dist %f and nr: %d\n",
             msg.x, msg.y, msg.z, min_dist, model_->getOutlierCount());
    if (model_->getOutlierCount() > 300) {
      model_->efficientRANSAC();
    }
  }
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept
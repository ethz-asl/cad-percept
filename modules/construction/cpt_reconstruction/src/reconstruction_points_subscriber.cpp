#include <cpt_reconstruction/reconstruction_points_subscriber.h>
#include <cpt_reconstruction/reconstruction_preprocess_model.h>

#include <pcl/point_types.h>
#include <cmath>
#include <sstream>
#include <string>
#include "cpt_reconstruction/coordinates.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace cad_percept {
namespace cpt_reconstruction {
ReconstructionPointsSubscriber::ReconstructionPointsSubscriber(
    ros::NodeHandle nodeHandle1, ros::NodeHandle nodeHandle2,
    PreprocessModel* model)
    : nodeHandle1_(nodeHandle1), nodeHandle2_(nodeHandle2), model_(model) {
  subscriber_ = nodeHandle1_.subscribe(
      "point", 1, &ReconstructionPointsSubscriber::messageCallback, this);
  publisher_ = nodeHandle2_.advertise<std_msgs::String>("ransac_shape", 10);
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
      // TODO: Forward RANSAC results
      std::stringstream ss;
      ss << "Shape ";
      std_msgs::String msg;
      msg.data = ss.str();
      publisher_.publish(msg);
    }
  }
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept
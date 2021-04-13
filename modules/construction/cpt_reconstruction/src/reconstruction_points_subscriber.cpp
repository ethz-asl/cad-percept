#include <cpt_reconstruction/reconstruction_points_subscriber.h>
#include <cpt_reconstruction/reconstruction_preprocess_model.h>

#include <geometry_msgs/Vector3.h>
#include <pcl/point_types.h>
#include <cmath>
#include <sstream>
#include <string>
#include "cpt_reconstruction/coordinates.h"
#include "cpt_reconstruction/shape.h"
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
  publisher_ =
      nodeHandle2_.advertise<::cpt_reconstruction::shape>("ransac_shape", 10);
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
    if (model_->getOutlierCount() > 500) {
      model_->clearRansacShapes();
      model_->efficientRANSAC();

      std::vector<Eigen::MatrixXd>* points_shape = model_->getPointShapes();
      std::vector<int>* shapes_ids = model_->getShapeIDs();

      for (int i = 0; i < shapes_ids->size(); i++) {
        std::vector<geometry_msgs::Vector3> pub_vectors;
        for (int j = 0; j < points_shape->at(i).cols(); j++) {
          geometry_msgs::Vector3 vec;
          vec.x = (*points_shape)[i](0, j);
          vec.y = (*points_shape)[i](1, j);
          vec.z = (*points_shape)[i](2, j);
          pub_vectors.push_back(vec);
        }
        ::cpt_reconstruction::shape msg;
        msg.vectors = pub_vectors;
        msg.id = shapes_ids->at(i);
        publisher_.publish(msg);
      }
    }
  }
}
}  // namespace cpt_reconstruction
}  // namespace cad_percept
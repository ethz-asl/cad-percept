#ifndef CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H
#define CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H

#include <cpt_reconstruction/reconstruction_preprocess_model.h>

#include "cpt_reconstruction/coordinates.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace cad_percept {
namespace cpt_reconstruction {
class ReconstructionPointsSubscriber {
 public:
  ReconstructionPointsSubscriber() = delete;
  ReconstructionPointsSubscriber(ros::NodeHandle nodeHandle,
                                 PreprocessModel* model);
  void startReceiving();

 private:
  void messageCallback(const ::cpt_reconstruction::coordinates& msg);
  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
  PreprocessModel* model_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H
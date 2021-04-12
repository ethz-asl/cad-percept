#ifndef CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H
#define CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H

#include "cpt_reconstruction/coordinates.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace cad_percept {
namespace cpt_reconstruction {
class ReconstructionPointsSubscriber {
 public:
  ReconstructionPointsSubscriber(ros::NodeHandle nodeHandle);
  void startReceiving();

 private:
  void messageCallback(const ::cpt_reconstruction::coordinates& msg);
  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_SUBSCRIBER_H
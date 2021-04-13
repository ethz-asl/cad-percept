#ifndef CPT_RECONSTRUCTION_MESHGENERATION_H
#define CPT_RECONSTRUCTION_MESHGENERATION_H

#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace cad_percept {
namespace cpt_reconstruction {
class MeshGeneration {
 public:
  MeshGeneration(ros::NodeHandle nodeHandle_);

 private:
  void messageCallback(const ::cpt_reconstruction::shape& msg);
  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept
#endif  // CPT_RECONSTRUCTION_MESHGENERATION_H

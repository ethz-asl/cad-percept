#ifndef CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H
#define CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H

#include <ros/ros.h>
#include <string>

namespace cad_percept {
namespace cpt_reconstruction {
class ReconstructionPointsPublisher {
 public:
  ReconstructionPointsPublisher() = delete;
  ReconstructionPointsPublisher(ros::NodeHandle nodeHandle, std::string filename, int batch_size);
  void publishPoints();

 private:
  ros::NodeHandle nodeHandle_;
  std::string filename_;
  int batch_size_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H

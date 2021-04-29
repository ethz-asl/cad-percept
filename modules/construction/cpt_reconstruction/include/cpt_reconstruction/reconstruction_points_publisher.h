#ifndef CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H
#define CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H

#include "cpt_reconstruction/coordinates.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cad_percept {
namespace cpt_reconstruction {
class ReconstructionPointsPublisher {
 public:
  ReconstructionPointsPublisher() = delete;
  ReconstructionPointsPublisher(ros::NodeHandle nodeHandle,
                                std::string filename, int batch_size);
  void publishPoints();

 private:
  ros::NodeHandle nodeHandle_;
  std::string filename_;
  int batch_size_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H

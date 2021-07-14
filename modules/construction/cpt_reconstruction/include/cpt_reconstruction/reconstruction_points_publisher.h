#ifndef CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H
#define CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H

#include <geometry_msgs/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <pcl/conversions.h>
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
  ReconstructionPointsPublisher(ros::NodeHandle nodeHandle);

  void publishPoints();

 private:
  ros::NodeHandle nodeHandle_;
  ros::Publisher publisher_;

  int SENSOR_TYPE_;
  std::string SCAN_PATH_FILE_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_RECONSTRUCTION_POINTS_PUBLISHER_H

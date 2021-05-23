#ifndef CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_
#define CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include "cpt_reconstruction/clusters.h"
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "cpt_reconstruction/coordinates.h"
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace cad_percept {
namespace cpt_reconstruction {
class MeshGeneration {
 public:
  MeshGeneration() = delete;
  MeshGeneration(ros::NodeHandle nodeHandle);

 private:
  void messageCallback(const ::cpt_reconstruction::shape &msg);

  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_

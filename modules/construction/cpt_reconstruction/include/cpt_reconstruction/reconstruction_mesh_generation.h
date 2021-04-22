#ifndef CPT_RECONSTRUCTION_MESHGENERATION_H
#define CPT_RECONSTRUCTION_MESHGENERATION_H

#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

namespace cad_percept {
namespace cpt_reconstruction {
class MeshGeneration {
 public:
  MeshGeneration(ros::NodeHandle nodeHandle_);

 private:
  void purgeBuffer();
  void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all);
  void messageCallback(const ::cpt_reconstruction::shape& msg);
  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
  int counter_;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> kd_trees_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_;
  std::vector<Eigen::Vector3d> ransac_normals_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept
#endif  // CPT_RECONSTRUCTION_MESHGENERATION_H

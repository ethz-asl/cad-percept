#ifndef CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_
#define CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include "cpt_reconstruction/classified_shapes.h"
#include "cpt_reconstruction/clusters.h"
#include "cpt_reconstruction/shape.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <cmath>
#include <exception>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/intersections.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace cad_percept {
namespace cpt_reconstruction {
class MeshGeneration {
 public:
  MeshGeneration() = delete;
  MeshGeneration(ros::NodeHandle nodeHandle);

 private:
  void messageCallback(const ::cpt_reconstruction::classified_shapes &msg);

  void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all);
  void preprocessBuildingModel();

  ros::NodeHandle nodeHandle_;
  ros::Subscriber subscriber_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_MESH_GENERATION_H_

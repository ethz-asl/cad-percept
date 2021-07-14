#ifndef CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_PROPOSAL_SELECTION_H_
#define CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_PROPOSAL_SELECTION_H_

#include "ros/ros.h"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <list>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>
#include <pcl/common/io.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace cad_percept {
namespace cpt_reconstruction {
class ProposalSelection {
 public:
  ProposalSelection() = delete;
  ProposalSelection(
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr upsampled_octree,
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> meshing_clouds,
      pcl::PolygonMesh mesh_model,
      pcl::PointCloud<pcl::PointXYZ>::Ptr model_upsampled_points,
      std::vector<Eigen::Vector3d> &center_estimates,
      std::vector<Eigen::Matrix3d> &direction_estimates,
      std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
      std::vector<std::vector<Eigen::VectorXd>> &parameter_hierarchies,
      std::vector<Eigen::MatrixXd> &bounded_axis_estimates,
      std::vector<double> &radius_estimates);

  void selectProposals();
  void organizeDatastructure();
  Eigen::VectorXd computePosterior(const Eigen::VectorXd &prior,
                                   const Eigen::VectorXd &posterior);
  void removeDuplicatedValues(std::vector<double> &vector, double eps);
  void upsampledStructuredPointCloud(
      double a1, double a2, double b1, double b2, double c1, double c2,
      Eigen::Vector3d center, Eigen::Matrix3d directions,
      pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud, double tol,
      double step = 0.03);
  double findParameterFromLikelihood(const Eigen::VectorXd &params,
                                     const Eigen::VectorXd &probabilities,
                                     int k);
  int findMaxIndexInEigenVector(const Eigen::VectorXd &vector);
  void removeConflictingElements();
  void removeInsufficientElements();
  void processModelPlanes();
  void L1Normalizer(Eigen::VectorXd &vec);

  void getSelectedProposals(
      std::vector<Eigen::Vector3d> &center_estimates,
      std::vector<Eigen::Matrix3d> &direction_estimates,
      std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates);

 private:
  // Planes
  std::vector<Eigen::Vector3d> center_estimates_;
  std::vector<Eigen::Matrix3d> direction_estimates_;
  std::vector<std::vector<Eigen::VectorXd>> parameter_estimates_;
  std::vector<std::vector<Eigen::VectorXd>> parameter_probabilities_;
  std::vector<std::vector<Eigen::VectorXd>> parameter_hierarchies_;

  // Cylinders
  std::vector<Eigen::MatrixXd> bounded_axis_estimates_;
  std::vector<double> radius_estimates_;

  // Remove Conflicting Shapes
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> structured_point_clouds_;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> kd_trees_;

  // Score computation
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr
      model_upsampled_octree_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr scan_kdtree_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr scan_octree_;

  // Incomplete Building Model
  pcl::PolygonMesh mesh_model_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_upsampled_points_;
  std::vector<double> mesh_plane_d_;
  std::vector<double> mesh_area_;
  std::vector<Eigen::Vector3d> mesh_plane_normals_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept

#endif  // CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_PROPOSAL_SELECTION_H_

#ifndef CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_PROPOSAL_SELECTION_H_
#define CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_PROPOSAL_SELECTION_H_

#include "ros/ros.h"

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>

namespace cad_percept {
namespace cpt_reconstruction {
class ProposalSelection {
 public:
  ProposalSelection() = delete;
  ProposalSelection(std::vector<Eigen::Vector3d> &center_estimates,
                    std::vector<Eigen::Matrix3d> &direction_estimates,
                    std::vector<std::vector<Eigen::VectorXd>> &parameter_estimates,
                    std::vector<Eigen::MatrixXd> &bounded_axis_estimates,
                    std::vector<double> &radius_estimates);

  void selectProposals();
  void organizeDatastructure();
  void removeConflictingElements();

  void getSelectedProposals();

 private:
  //Planes
  std::vector<Eigen::Vector3d> center_estimates_;
  std::vector<Eigen::Matrix3d> direction_estimates_;
  std::vector<std::vector<Eigen::VectorXd>> parameter_estimates_;

  //Cylinders
  std::vector<Eigen::MatrixXd> bounded_axis_estimates_;
  std::vector<double> radius_estimates_;

  // Remove Conflicting Shapes
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> structured_point_clouds_;
  std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> kd_trees_;

};
}
}

#endif //CPT_RECONSTRUCTION_SRC_RECONSTRUCTION_PROPOSAL_SELECTION_H_

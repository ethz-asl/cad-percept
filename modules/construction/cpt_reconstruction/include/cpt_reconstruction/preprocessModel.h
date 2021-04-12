#ifndef CPT_RECONSTRUCTION_PREPROCESSMODEL_H
#define CPT_RECONSTRUCTION_PREPROCESSMODEL_H

#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace cad_percept {
namespace cpt_reconstruction {
class PreprocessModel {
 public:
  PreprocessModel() = delete;
  PreprocessModel(std::string filename, Eigen::Matrix4d transformation);
  void preprocess();
  float queryTree(pcl::PointXYZ p);
  void addOutlier(float i);
  void printOutliers();
  int getOutlierCount();

 private:
  Eigen::Matrix4d transformation_;
  std::string filename_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_points_;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree_;
  std::vector<int> nn_indices_{1};
  std::vector<float> nn_dists_{1};
  std::vector<float> dist_outlier_;
};
}  // namespace cpt_reconstruction
}  // namespace cad_percept
#endif  // CPT_RECONSTRUCTION_PREPROCESSMODEL_H

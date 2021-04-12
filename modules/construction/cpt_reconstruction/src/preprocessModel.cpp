#include <cpt_reconstruction/preprocessModel.h>

#include "ros/ros.h"

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
PreprocessModel::PreprocessModel(std::string filename, Eigen::Matrix4d transformation) {
  filename_ = filename;
  transformation_ = transformation;
}

void PreprocessModel::preprocess() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  reader.read(filename_, *model_points);
  model_points_ = model_points;

  // Build Search Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree(new pcl::search::KdTree<pcl::PointXYZ>());
  searchTree->setInputCloud(model_points_);
  searchTree_ = searchTree;
}

float PreprocessModel::queryTree(pcl::PointXYZ p) {
  searchTree_->nearestKSearch(p, 1, nn_indices_, nn_dists_);
  return nn_dists_[0];
}

void PreprocessModel::addOutlier(float i) { dist_outlier_.push_back(i); }

void PreprocessModel::printOutliers() {
  for (unsigned i = 0; i < dist_outlier_.size(); i++) {
    ROS_INFO("Outlier idx: %f\n", dist_outlier_.at(i));
  }
}
int PreprocessModel::getOutlierCount() { return dist_outlier_.size(); }

}  // namespace cpt_reconstruction
}  // namespace cad_percept

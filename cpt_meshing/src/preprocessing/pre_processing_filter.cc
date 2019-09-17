#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <cpt_meshing/preprocessing/pre_processing_filter.h>

namespace cad_percept {
namespace meshing {

PreProcessingFilter::PreProcessingFilter()
    : normal_search_radius_(0.03)  // set reasonable default
{}

void PreProcessingFilter::run(cad_percept::meshing::InputPointCloud::ConstPtr input,
                              cad_percept::meshing::InputPointCloud::Ptr output,
                              cad_percept::meshing::InputNormals::Ptr normals) {
  // run through filters naively
  pcl::copyPointCloud(*input, *output);
  for (auto filter : filter_list_) {
    // In place filter always creates a cache copy - maybe avoid in future.
    filter->setInputCloud(output);
    filter->filter(*output);
  }

  // estimate normals
  estimateNormals(output, normals);
}

void PreProcessingFilter::addBoxFilter(const std::string& axis, const double min,
                                       const double max) {
  std::shared_ptr<pcl::PassThrough<InputPoint>> filter =
      std::make_shared<pcl::PassThrough<InputPoint>>();
  filter->setFilterFieldName(axis);
  filter->setFilterLimits(min, max);
  filter_list_.push_back(filter);
}

void PreProcessingFilter::addVoxelFilter(const double x_leaf, const double y_leaf,
                                         const double z_leaf) {
  std::shared_ptr<pcl::VoxelGrid<InputPoint>> filter =
      std::make_shared<pcl::VoxelGrid<InputPoint>>();

  filter->setLeafSize(x_leaf, y_leaf, z_leaf);
  filter_list_.push_back(filter);
}

void PreProcessingFilter::estimateNormals(cad_percept::meshing::InputPointCloud::ConstPtr input,
                                          cad_percept::meshing::InputNormals::Ptr normals) {
  pcl::NormalEstimation<InputPoint, pcl::Normal> ne;
  ne.setInputCloud(input);

  pcl::search::KdTree<InputPoint>::Ptr tree(new pcl::search::KdTree<InputPoint>());
  ne.setSearchMethod(tree);

  ne.setRadiusSearch(normal_search_radius_);
  ne.compute(*normals);
}
}  // namespace meshing
}  // namespace cad_percept

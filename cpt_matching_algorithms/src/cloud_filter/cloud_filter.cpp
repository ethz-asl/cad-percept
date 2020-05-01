#include "cloud_filter/cloud_filter.h"

namespace cad_percept {
namespace matching_algorithms {

using namespace pcl;

void CloudFilter::filterStaticObject(int structure_threshold, PointCloud<PointXYZ>& lidar_scan,
                                     const PointCloud<PointXYZI> static_structure) {
  std::cout << "////  Static Object Filter  ////" << std::endl;

  ExtractIndices<pcl::PointXYZ> indices_filter;

  if (lidar_scan.size() != static_structure.size()) {
    std::cout << "Error: The sizes of the two point cloud do not match" << std::endl;
    std::cout << "Skip Static Object Filter" << std::endl;
    return;
  }

  std::vector<int> rm_idx;
  for (int idx = 0; idx < lidar_scan.size(); ++idx) {
    if (static_structure.points[idx].intensity < structure_threshold) {
      rm_idx.push_back(idx);
    }
  }

  PointCloud<pcl::PointXYZ>::Ptr lidar_scan_ptr(new PointCloud<PointXYZ>());
  *lidar_scan_ptr = lidar_scan;
  boost::shared_ptr<std::vector<int>> inliers_ptr = boost::make_shared<std::vector<int>>(rm_idx);
  indices_filter.setInputCloud(lidar_scan_ptr);
  indices_filter.setIndices(inliers_ptr);
  indices_filter.setNegative(true);
  indices_filter.filter(*lidar_scan_ptr);
  lidar_scan = *lidar_scan_ptr;

  std::cout << "Lidar frame filtered by Static Object Filter with threshold " << structure_threshold
            << std::endl;
  std::cout << "Point Cloud size: " << lidar_scan.size()
            << " Removed: " << static_structure.size() - lidar_scan.size() << std::endl;
}

void CloudFilter::filterVoxelCentroid(float search_radius, PointCloud<PointXYZ>& lidar_scan) {
  // std::cout << "////    Voxel Centroid Filter   ////" << std::endl;
  int prev_size = lidar_scan.size();

  PointCloud<PointXYZ>::Ptr lidar_scan_ptr(new PointCloud<PointXYZ>());
  *lidar_scan_ptr = lidar_scan;
  UniformSampling<PointXYZ> voxel_filter;
  voxel_filter.setInputCloud(lidar_scan_ptr);
  voxel_filter.setRadiusSearch(search_radius);
  voxel_filter.filter(*lidar_scan_ptr);
  lidar_scan = *lidar_scan_ptr;

  // std::cout << "Point Cloud size: " << lidar_scan.size()
  //           << " Removed: " << prev_size - lidar_scan.size() << std::endl;
}
}  // namespace matching_algorithms
}  // namespace cad_percept
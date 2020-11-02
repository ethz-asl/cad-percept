#ifndef CLOUD_FILTER_H_
#define CLOUD_FILTER_H_

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/point_types.h>
#include <iostream>
#include <vector>

namespace cad_percept {
namespace matching_algorithms {

class CloudFilter {
 public:
  // Filters out points with static value under threshold
  static void filterStaticObject(int structure_threshold,
                                 pcl::PointCloud<pcl::PointXYZ>& lidar_scan,
                                 const pcl::PointCloud<pcl::PointXYZI> static_structure);
  // Replaces points in one voxel with average point of all points
  static void filterVoxelCentroid(float search_radius, pcl::PointCloud<pcl::PointXYZ>& lidar_scan);
};
}  // namespace matching_algorithms
}  // namespace cad_percept

#endif
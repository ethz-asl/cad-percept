#ifndef CLOUD_FILTER_H_
#define CLOUD_FILTER_H_

#include <pcl/point_types.h>
#include <ros/ros.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/uniform_sampling.h>

namespace cad_percept {
namespace matching_algorithms {

class CloudFilter {
 public:
  static void filterStaticObject(int structure_threshold,
                                 pcl::PointCloud<pcl::PointXYZ>& lidar_scan,
                                 const pcl::PointCloud<pcl::PointXYZI> static_structure);
  static void filterVoxelCentroid(float search_radius, pcl::PointCloud<pcl::PointXYZ>& lidar_scan);
};
}  // namespace matching_algorithms
}  // namespace cad_percept

#endif
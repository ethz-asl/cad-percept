#include <pcl/point_types.h>
#include <ros/ros.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/uniform_sampling.h>

class CloudFilterLib {
 public:
  static void static_object_filter(int structure_threshold,
                                   pcl::PointCloud<pcl::PointXYZ>& lidar_scan,
                                   const pcl::PointCloud<pcl::PointXYZI> static_structure);
  static void voxel_centroid_filter(float search_radius,
                                    pcl::PointCloud<pcl::PointXYZ>& lidar_scan);
};

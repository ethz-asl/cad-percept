#include <pcl/point_types.h>
#include <ros/ros.h>

#include <pcl/filters/extract_indices.h>

class CloudFilterLib {
 public:
  static void static_object_filter(int structure_threshold,
                                   pcl::PointCloud<pcl::PointXYZ>& lidar_frame,
                                   const pcl::PointCloud<pcl::PointXYZI> static_structure);
};

#ifndef DESCRIPTOR_MATCHER_H_
#define DESCRIPTOR_MATCHER_H_

#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <fstream>
#include <random>

namespace cad_percept {
namespace matching_algorithms {

typedef Eigen::Matrix<float, 4, 4> Transformation;
typedef std::vector<Transformation, Eigen::aligned_allocator<Transformation>> TransformationVector;

class StruDe {
 public:
  StruDe();
  // Returns transformation for alignment using Structural Descriptors
  visualization_msgs::Marker strudeMatch(Eigen::Matrix4d& res_transform,
                                         const pcl::PointCloud<pcl::PointXYZ>& lidar_scan,
                                         const pcl::PointCloud<pcl::PointXYZ>& sampled_map);

  visualization_msgs::Marker matchesToRosMsg(
      const pcl::PointCloud<pcl::PointSurfel>::Ptr& keypoints_scan,
      const pcl::PointCloud<pcl::PointSurfel>::Ptr& keypoints_map,
      const pcl::Correspondences& correspondences) const;

  bool map_computed_;
  pcl::PointCloud<pcl::PointSurfel>::Ptr keypoints_map_;
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptors_map_;
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif
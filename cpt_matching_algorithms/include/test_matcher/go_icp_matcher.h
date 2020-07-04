#ifndef GO_ICP_MATCHER_H_
#define GO_ICP_MATCHER_H_

#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <fstream>
#include <random>

namespace cad_percept {
namespace matching_algorithms {
class GoIcp {
 public:
  // Returns transformation for alignment using GoICP
  static void goIcpMatch(Eigen::Matrix4d &res_transform,
                         const pcl::PointCloud<pcl::PointXYZ> &lidar_scan,
                         const pcl::PointCloud<pcl::PointXYZ> &sampled_map);
};

}  // namespace matching_algorithms
}  // namespace cad_percept

#endif
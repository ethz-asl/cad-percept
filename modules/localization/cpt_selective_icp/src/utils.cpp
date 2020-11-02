#include "cpt_selective_icp/utils.h"

namespace cad_percept {
namespace selective_icp {
namespace utils {

PointCloud dpToPointCloud(const DP &dppointcloud) {
  const size_t n_points = dppointcloud.getNbPoints();
  PointCloud cloud;
  for (uint i = 0; i < n_points; ++i) {
    pcl::PointXYZ point;
    point.x = dppointcloud.features(0, i);
    point.y = dppointcloud.features(1, i);
    point.z = dppointcloud.features(2, i);
    cloud.push_back(point);
  }
  return cloud;
}

DP pointCloudToDP(const PointCloud &pointcloud) {
  // alternatively rosMsgToPointMatcherCloud (libpointmatcher_ros)
  const int dimFeatures = 4;

  PM::Matrix feat(dimFeatures, pointcloud.points.size());
  for (uint i = 0; i < pointcloud.points.size(); ++i) {
    feat(0, i) = pointcloud[i].x;
    feat(1, i) = pointcloud[i].y;
    feat(2, i) = pointcloud[i].z;
    feat(3, i) = 1.0;
  }

  DP::Labels featLabels;
  featLabels.push_back(DP::Label("x", 1));
  featLabels.push_back(DP::Label("y", 1));
  featLabels.push_back(DP::Label("z", 1));
  featLabels.push_back(DP::Label("pad", 1));

  DP dppointcloud = DP(
      feat, featLabels);  // construct a point cloud from existing features without any descriptor

  return dppointcloud;
}

}  // namespace utils
}  // namespace selective_icp
}  // namespace cad_percept
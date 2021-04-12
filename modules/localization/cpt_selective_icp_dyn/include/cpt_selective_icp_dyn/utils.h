#ifndef CPT_SELECTIVE_ICP_DYN_UTILS_H_
#define CPT_SELECTIVE_ICP_DYN_UTILS_H_

#include <cgal_definitions/cgal_typedefs.h>
#include <pcl_ros/point_cloud.h>
#include "pointmatcher/PointMatcher.h"

namespace cad_percept {
namespace selective_icp_dyn {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace utils {

/*
 * Convert a DP to a PointCloud
 */
DP pointCloudToDP(const PointCloud &pointcloud);

/*
 * Convert a PointCloud to a DP
 */
PointCloud dpToPointCloud(const DP &dppointcloud);

}  // namespace utils
}  // namespace selective_icp_dyn
}  // namespace cad_percept
#endif
#include <cgal_definitions/cgal_typedefs.h>
#include <pcl_ros/point_cloud.h>
#include "pointmatcher/PointMatcher.h"

namespace cad_percept {
namespace selective_icp {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

namespace utils {
DP pointCloudToDP(const PointCloud &pointcloud);

PointCloud dpToPointCloud(const DP &dppointcloud);

}  // namespace utils
}  // namespace selective_icp
}  // namespace cad_percept
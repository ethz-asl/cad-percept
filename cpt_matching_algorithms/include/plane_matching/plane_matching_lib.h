#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pointmatcher_ros/point_cloud.h>

class PlaneMatchLib {
 public:
  static void prrus(float (&transfromTR)[7], const pcl::PointCloud<pcl::PointNormal> scan_planes,
                    const pcl::PointCloud<pcl::PointNormal> map_planes);

 private:
  static void transform_split(float (&transfromTR)[7], std::vector<int> plane_assignement,
                              const pcl::PointCloud<pcl::PointNormal> scan_planes,
                              const pcl::PointCloud<pcl::PointNormal> map_planes);
};

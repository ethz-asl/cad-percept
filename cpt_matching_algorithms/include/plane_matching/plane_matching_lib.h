#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <Eigen/Eigenvalues>

class PlaneMatchLib {
 public:
  static void prrus(float (&transformTR)[7], const pcl::PointCloud<pcl::PointNormal> scan_planes,
                    const pcl::PointCloud<pcl::PointNormal> map_planes,
                    ros::NodeHandle &nh_private);
  static void load_example_sol(float (&transformTR)[7],
                               const pcl::PointCloud<pcl::PointNormal> scan_planes,
                               const pcl::PointCloud<pcl::PointNormal> map_planes,
                               ros::NodeHandle &nh_private);

 private:
  static void transform_average(float (&transformTR)[7], std::vector<int> plane_assignement,
                                const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                const pcl::PointCloud<pcl::PointNormal> map_planes);
};

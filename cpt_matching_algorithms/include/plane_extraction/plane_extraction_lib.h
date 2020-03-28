// PlaneExtractionLib.h

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/geometry.h>
#include <pcl/kdtree/kdtree_flann.h>

class PlaneExtractionLib {
 public:
  class HoughAccumulator;
  class ArrayAccumulator;
  class BallAccumulator;

  static std::vector<double> rht_plane_extraction(const pcl::PointCloud<pcl::PointXYZ> lidar_frame,
                                                  ros::Publisher &plane_pub,
                                                  std::string tf_map_frame,
                                                  ros::NodeHandle &nh_private);
  static std::vector<double> iterative_rht_plane_extraction(
      const pcl::PointCloud<pcl::PointXYZ> lidar_frame, ros::Publisher &plane_pub,
      std::string tf_map_frame, ros::NodeHandle &nh_private);
  static std::vector<double> pcl_plane_extraction(const pcl::PointCloud<pcl::PointXYZ> lidar_frame,
                                                  ros::Publisher &plane_pub,
                                                  std::string tf_map_frame,
                                                  ros::NodeHandle &nh_private);
  static void visualize_plane(std::vector<pcl::PointCloud<pcl::PointXYZRGB>> &extracted_planes,
                              ros::Publisher &plane_pub, std::string tf_map_frame);
};

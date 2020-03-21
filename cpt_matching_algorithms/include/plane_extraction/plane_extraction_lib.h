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

class PlaneExtractionLib {
 public:
  static std::vector<pcl::PointCloud<pcl::PointXYZRGB>> pcl_plane_extraction(
      const pcl::PointCloud<pcl::PointXYZ> lidar_frame, int max_number_of_plane,
      int min_number_of_inlier, ros::Publisher &plane_pub_, std::string tf_map_frame);
};

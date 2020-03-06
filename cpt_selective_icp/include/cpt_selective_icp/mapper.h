#ifndef SELECTIVE_ICP_MAPPER_H_
#define SELECTIVE_ICP_MAPPER_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/ColoredMesh.h>
#include <cgal_msgs/ReferenceTask.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/pc_processing.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher/Timer.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/thread.hpp>
#include <unordered_set>
#include "cpt_selective_icp/BuildingTask.h"
#include "cpt_selective_icp/References.h"
#include "cpt_selective_icp/mapper_parameters.h"

namespace cad_percept {
namespace selective_icp {

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Mapper {
 public:
  Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 protected:
  ros::NodeHandle &nh_, &nh_private_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  MapperParameters parameters_;
  ros::Time stamp;
  int odom_received_;

  /**
   * Reference mesh for localization
   */
  cgal::MeshModel::Ptr reference_mesh_;
  PM::TransformationParameters T_map_to_meshorigin_;
  std::string mesh_frame_id_;

  /**
   * Set containing all reference IDs and their coplanar neighbors.
   */
  std::unordered_set<std::string> all_coplanar_references;

  // libpointmatcher
  PM::ICPSequence icp_;
  PM::ICPSequence selective_icp_;
  PM::DataPointsFilters input_filters_;
  PM::DataPointsFilters map_pre_filters_;
  PM::DataPointsFilters map_post_filters_;
  PM::TransformationParameters T_scanner_to_map_;
  std::shared_ptr<PM::Transformation> transformation_;

  // Time
  ros::Time last_point_cloud_time_;
  uint32_t last_point_cloud_seq_;

  bool cad_trigger;
  bool selective_icp_trigger;
  bool full_icp_trigger;
  bool ref_mesh_ready;
  int projection_count;
  DP mapPointCloud;
  bool mapping_trigger;
  bool update_icp_ref_trigger;

  DP ref_dp;
  DP selective_ref_dp;

  boost::thread map_thread;

  // Subscribers
  ros::Subscriber cloud_sub_;
  ros::Subscriber cad_sub_;

  // Publishers
  ros::Publisher ref_mesh_pub_;
  ros::Publisher cad_mesh_pub_;
  ros::Publisher ref_pc_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher mesh_pose_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher scan_pub_;
  ros::Publisher selective_icp_scan_pub_;
  ros::Publisher point_pub_;
  ros::Publisher map_pub_;

  // Services
  ros::ServiceServer load_published_map_srv_;
  ros::ServiceServer get_closest_facet_srv_;
  ros::ServiceServer set_ref_srv_;
  ros::ServiceServer set_full_icp_srv_;
  ros::ServiceServer set_selective_icp_srv_;
  ros::ServiceServer reload_icp_config_srv_;

  /**
   * Point cloud callback
   */
  void gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in);

  /**
   * Mesh model callback
   */
  void gotCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);

  /**
   * Get some sort of residual error, but only for points associated to our references.
   * Apply threshold first to avoid taking into account points from other walls.
   * Attention: These also take into account non-wall points and objects.
   */
  double getICPErrorToRef(const DP &aligned_dp);

  /**
   * Get some sort of residual error
   * Apply threshold first to avoid taking into account points from other walls.
   */
  double getICPError(const DP &aligned_dp);

  /**
   * Computation of different ICP error metrics:
   * - Residual Error
   * - Robust Mean Distance
   * - Hausdorff distance
   */
  void getError(DP dpref, DP dppointcloud_out, bool selective);

  /**
   * Filtering of point cloud, general pre-processing of dp cloud for both
   * map and reading cloud.
   */
  void processCloud(DP *point_cloud, const ros::Time &stamp);

  /**
   * Perform ICP
   */
  bool selectiveICP(const DP &cloud, PM::TransformationParameters *T_updated_scanner_to_map,
                    const ros::Time &stamp);
  bool fullICP(const DP &cloud, PM::TransformationParameters *T_updated_scanner_to_map);

  /*
   * Service Calls for dynamic settings
   */
  bool setReferenceFacets(cpt_selective_icp::References::Request &req,
                          cpt_selective_icp::References::Response &res);
  bool setReferenceTask(cpt_selective_icp::BuildingTask::Request &req,
                        cpt_selective_icp::BuildingTask::Response &res);
  bool setFullICP(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool setSelectiveICP(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

  template <class T>
  void publishCloud(T *cloud, ros::Publisher *publisher) const;

  /**
   * Publish reference mesh with all reference triangles in red.
   */
  void publishReferenceMesh(const std::unordered_set<std::string> &references);

  /**
   * Sample a point cloud from selected triangles of the mesh model.
   */
  void sampleFromReferenceFacets(const int density, std::unordered_set<std::string> &references,
                                 PointCloud *pointcloud);

  /**
   * Set a trigger to load a newly published mesh as map.
   */
  bool loadPublishedMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * Load parameters for libpointmatcher from .yaml.
   */
  void loadConfig();
  bool reloadConfig(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * Add a new aligned scan to the map.
   */
  void addScanToMap(DP &corrected_cloud, ros::Time &stamp);
};

}  // namespace selective_icp
}  // namespace cad_percept

#endif  // SELECTIVE_ICP_MAPPER_H_

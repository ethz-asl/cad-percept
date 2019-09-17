#ifndef SELECTIVE_ICP_MAPPER_H_
#define SELECTIVE_ICP_MAPPER_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_utils/pc_processing.h>
#include <pcl/point_types.h>
#include <pointmatcher/PointMatcher.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/bimap.hpp>
#include <boost/bimap/unordered_multiset_of.hpp>
#include <boost/bimap/unordered_set_of.hpp>
#include <unordered_set>
#include "cpt_selective_icp/mapper_parameters.h"

#include <cgal_msgs/ColoredMesh.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include "cpt_selective_icp/FacetID.h"
#include "cpt_selective_icp/References.h"

#include <pointmatcher/Timer.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>

#include <boost/thread.hpp>

namespace cad_percept {
namespace selective_icp {

typedef boost::bimap<boost::bimaps::unordered_set_of<int>,
                     boost::bimaps::unordered_multiset_of<int>>
    association_bimap;
typedef association_bimap::value_type bi_association;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::map<int, int>::iterator Miterator;
typedef std::multimap<int, int>::iterator Mmiterator;

class Mapper {
 public:
  Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~Mapper();

 private:
  ros::NodeHandle &nh_, &nh_private_;
  tf::TransformListener tf_listener_;
  MapperParameters parameters_;
  std::ofstream timingFile;
  std::ofstream metricsFile;
  std::ofstream transformationFile;
  std::ofstream transformationFile2;
  cgal::MeshModel reference_mesh_;
  ros::Time stamp;
  void gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in);
  void gotCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);
  /**
   * Get some sort of residual error, but only for points associated to our references.
   * Apply threshold first to avoid taking into account points from other walls.
   * Attention: These also take into account non-wall points and objects
   */
  double getICPErrorToRef(const DP &aligned_dp);
  /**
   * Get some sort of residual error
   * Apply threshold first to avoid taking into account points from other walls/ assuming we have a
   * certain initial transformation... check what distance is appropriate max after initial
   * transformation
   */
  double getICPError(const DP &aligned_dp);
  void getError(DP dpref, DP dppointcloud_out, bool selective);
  void processCloud(DP *point_cloud, const ros::Time &stamp);
  bool selectiveICP(const DP &cloud, PM::TransformationParameters *T_updated_scanner_to_map,
                    const ros::Time &stamp);
  bool normalICP(const DP &cloud, PM::TransformationParameters *T_updated_scanner_to_map);
  bool setReferenceFacets(cpt_selective_icp::References::Request &req,
                          cpt_selective_icp::References::Response &res);
  bool setNormalICP(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool setSelectiveICP(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  void publishReferenceMesh(cgal::MeshModel &reference_mesh, std::unordered_set<int> &references);
  template <class T>
  void publishCloud(T *cloud, ros::Publisher *publisher) const;
  void extractReferenceFacets(const int density, cgal::MeshModel &reference_mesh,
                              std::unordered_set<int> &references, PointCloud *pointcloud);
  std::unordered_set<int> references_new;

  bool loadPublishedMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool getClosestFacet(cpt_selective_icp::FacetID::Request &req,
                       cpt_selective_icp::FacetID::Response &res);
  /**
   * Load parameters for libpointmatcher from yaml
   */
  void loadConfig();
  bool reloadConfig(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void addScanToMap(DP &corrected_cloud, ros::Time &stamp);

  void publishMesh(const cgal::MeshModel &model, ros::Publisher *publisher) const;

  // Subscribers
  ros::Subscriber cloud_sub_;
  ros::Subscriber cad_sub_;

  // Publishers
  ros::Publisher ref_mesh_pub_;
  ros::Publisher cad_mesh_pub_;
  ros::Publisher ref_pc_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher scan_pub_;
  ros::Publisher selective_icp_scan_pub_;
  ros::Publisher point_pub_;
  ros::Publisher map_pub_;

  // Services
  ros::ServiceServer load_published_map_srv_;
  ros::ServiceServer get_closest_facet_srv_;
  ros::ServiceServer set_ref_srv_;
  ros::ServiceServer set_normal_icp_srv_;
  ros::ServiceServer set_selective_icp_srv_;
  ros::ServiceServer reload_icp_config_srv_;

  int odom_received_;

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
  bool normal_icp_trigger;
  bool ref_mesh_ready;
  int projection_count;
  DP mapPointCloud;
  bool mapping_trigger;
  bool update_icp_ref_trigger;

  DP ref_dp;
  DP selective_ref_dp;

  boost::thread map_thread;
};

}  // namespace selective_icp
}  // namespace cad_percept

#endif  // SELECTIVE_ICP_MAPPER_H_
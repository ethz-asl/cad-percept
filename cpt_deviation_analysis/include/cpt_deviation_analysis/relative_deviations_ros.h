/**
 * Deviation Analysis
 * This is the real-time deviation analysis. ICP is executed in separate 
 * package cpt_selective_icp.
 */

#ifndef RELATIVE_DEVIATIONS_ROS_H_
#define RELATIVE_DEVIATIONS_ROS_H_

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_msgs/ColoredMesh.h>
#include <cgal_msgs/GeomDeviation.h>
#include <cgal_msgs/SetDeviationPlane.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/conversions.h>
#include <cpt_utils/cpt_utils.h>
#include <cpt_utils/pc_processing.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <glog/logging.h>
#include <pcl/filters/random_sample.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/circular_buffer.hpp>
#include <map>
#include <unordered_map>
#include "cpt_deviation_analysis/deviations.h"

namespace cad_percept {
namespace deviations {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> ColoredPointCloud;

class RelativeDeviations {
 public:
  RelativeDeviations(ros::NodeHandle &nh, ros::NodeHandle &nh_private);

 private:
  Deviations deviations;
  ros::NodeHandle &nh_, nh_private_;
  tf::TransformListener tf_listener_;

  /**
   * Publish a mesh model.
   */
  void publishMesh(const cgal::MeshModel::Ptr &model, ros::Publisher *publisher) const;
  
  /**
   * Publish a point cloud.
   */
  template <class T>
  void publishCloud(T *cloud, ros::Publisher *publisher) const;

  // Publishers:
  ros::Publisher buffer_pc_pub_, reconstructed_planes_pub_, polygon_pub_, assoc_mesh_pub_,
      assoc_pc_pub_, assoc_marker_pub_, bboxes_marker_pub_, deviations_mesh_pub_,
      mesh_normals_marker_pub_, all_mesh_normals_marker_pub_, deviations_pub_;
  
  // Services:
  ros::ServiceServer analyze_map_srv_;

  std::string map_frame_;
  bool discrete_color_;
  float score_threshold_;

  /**
   * Publish point cloud of segmented planes
   */
  void publishReconstructedPlanes(const std::vector<reconstructed_plane> &rec_planes,
                                  ros::Publisher *publisher) const;
  /**
   *  Visualize boundaries of each facet. Currently there is no use for this anymore, 
   *  since we do not merge anymore and just use triangle meshes, which can be 
   *  visualized.
   */
  void publishPolyhedron(cgal::Polyhedron &P);

  /**
   * Process a new reading point cloud.
   */
  void processCloud(cgal::PointCloud &reading_pc);

  /**
   * Process a complete map.
   */
  void processMap(PointCloud &map_pc);

  /**
   * A circular_buffer to store aligned reading pointclouds.
   */
  boost::circular_buffer<cgal::PointCloud> cb;

  /**
   * Buffer is processed starting the deviation analysis.
   */
  void processBuffer(cgal::PointCloud &reading_pc);

  /**
   * Visualization publisher function.
   */
  void publish(const std::vector<reconstructed_plane> &rec_planes,
               const std::vector<reconstructed_plane> &remaining_plane_cloud_vector,
               std::unordered_map<std::string, transformation> &transformation_map);

  /**
   * Visualize segmented plane to model plane associations.
   */
  void publishAssociations(const cgal::MeshModel::Ptr &model,
                           std::unordered_map<std::string, polyhedron_plane> &plane_map,
                           const std::vector<reconstructed_plane> &remaining_plane_cloud_vector);
  /**
   * Visualize bounding boxes and normals of segmented plane.
   */
  void publishBboxesAndNormals(std::unordered_map<std::string, polyhedron_plane> &plane_map);

  /**
   * Visualize only associated model normals.
   */
  void publishModelNormals(std::unordered_map<std::string, polyhedron_plane> &plane_map);

  /**
   * Visualize all model normals.
   */
  void publishAllModelNormals(std::unordered_map<std::string, polyhedron_plane> &plane_map);

  /**
   * Visualize deviations.
   */
  void publishDeviations(const cgal::MeshModel::Ptr &model,
                         std::unordered_map<std::string, transformation> &transformation_map);

  // Selection of plane to publish deviation
  std::string selected_facet_;
  std::string current_task_id_;
  ros::ServiceServer set_deviation_plane_;
  bool deviationTargetServiceCallback(cgal_msgs::SetDeviationPlane::Request &req,
                                      cgal_msgs::SetDeviationPlane::Response &resp);

  std::string cad_topic;
  std::string scan_topic;
  std::string map_topic;
  int input_queue_size;
  int map_freq;
  std::string visualize;
  bool map_analyzer_trigger;

  // Subscribers
  ros::Subscriber cad_sub_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber map_sub_;
  std::string cad_frame_;  // remember frame of CAD model

  /**
   * Processing of a new CAD model.
   */
  void gotCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);

  /** 
   * Processing of a new point cloud.
   */
  void gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in);
  
  /**
   * Processing of a new map.
   */
  void gotMap(const sensor_msgs::PointCloud2 &cloud_msg_in);

  /**
   * Setting trigger for map analysis.
   */
  bool analyzeMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
   * A map saving colors associated to planes for more uniform visualization.
   */
  std::unordered_map<std::string, std_msgs::ColorRGBA> c_associated_;
};

}  // namespace deviations
}  // namespace cad_percept

#endif  // RELATIVE_DEVIATIONS_ROS_H_

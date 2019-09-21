#ifndef RELATIVE_DEVIATIONS_ROS_H_
#define RELATIVE_DEVIATIONS_ROS_H_

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_msgs/ColoredMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_utils/cpt_utils.h>
#include <cpt_utils/pc_processing.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <glog/logging.h>
#include <pcl/filters/random_sample.h>
#include <pcl_ros/point_cloud.h>
#include <pointmatcher/Timer.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/Empty.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "cpt_deviation_analysis/deviations.h"

#include <boost/circular_buffer.hpp>

#include <map>
#include <unordered_map>

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
  std::ofstream timingFile;
  void publishMesh(const cgal::MeshModel &model, ros::Publisher *publisher) const;
  template <class T>
  void publishCloud(T *cloud, ros::Publisher *publisher) const;
  ros::Publisher buffer_pc_pub_, reconstructed_planes_pub_, polygon_pub_, assoc_mesh_pub_,
      assoc_pc_pub_, assoc_marker_pub_, bboxes_marker_pub_, deviations_mesh_pub_,
      mesh_normals_marker_pub_, all_mesh_normals_marker_pub_;
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
   *  Currently there is no use for this anymore, since we do not merge
   *  anymore and just use triangle meshes.
   */
  void publishPolyhedron(cgal::Polyhedron &P);
  void processCloud(cgal::PointCloud &reading_pc);
  void processMap(PointCloud &map_pc);
  // create a circular_buffer to store reading pointclouds for alignment
  boost::circular_buffer<cgal::PointCloud> cb;
  void processBuffer(cgal::PointCloud &reading_pc);

  void publish(const std::vector<reconstructed_plane> &rec_planes,
               const std::vector<reconstructed_plane> &remaining_plane_cloud_vector,
               std::unordered_map<int, transformation> &transformation_map);

  void publishAssociations(const cgal::MeshModel &model,
                           std::unordered_map<int, polyhedron_plane> &plane_map,
                           const std::vector<reconstructed_plane> &remaining_plane_cloud_vector);
  void publishBboxesAndNormals(std::unordered_map<int, polyhedron_plane> &plane_map);
  void publishModelNormals(std::unordered_map<int, polyhedron_plane> &plane_map);
  void publishAllModelNormals(std::unordered_map<int, polyhedron_plane> &plane_map);
  void publishDeviations(const cgal::MeshModel &model,
                         std::unordered_map<int, transformation> &transformation_map);

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

  void gotCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);
  void gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in);
  void gotMap(const sensor_msgs::PointCloud2 &cloud_msg_in);
  bool analyzeMap(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  std::vector<std_msgs::ColorRGBA> c_associated;  // keep association colors in current callback
};

}  // namespace deviations
}  // namespace cad_percept

#endif  // RELATIVE_DEVIATIONS_ROS_H_

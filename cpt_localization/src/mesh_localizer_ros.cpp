#include <pcl_conversions/pcl_conversions.h>
#include <tf_conversions/tf_eigen.h>

#include "cpt_localization/mesh_localizer_ros.h"

namespace cad_percept {
namespace localization {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

MeshLocalizerRos::MeshLocalizerRos(ros::NodeHandle &nh,
                                   ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      mesh_localizer_(nh_private.param<std::string>("off_model", "fail")),
      map_frame_(nh_private.param<std::string>("map_frame", "fail")),
      cad_frame_(nh_private.param<std::string>("cad_frame", "fail")),
      distance_threshold_(nh_private.param<double>("distance_threshold", 0.2)) {
  if (!nh_private_.hasParam("off_model"))
    std::cerr << "ERROR 'off_model' not set as parameter." << std::endl;
  good_matches_pub_ =
      nh_.advertise<visualization_msgs::Marker>("good_cad_matches", 100);
  bad_matches_pub_ =
      nh_.advertise<visualization_msgs::Marker>("bad_cad_matches", 100);
  model_pub_ =
      nh_.advertise<visualization_msgs::Marker>("architect_model", 100);
  arch_pub_ =
      nh_.advertise<PointCloud>("architect_model_pcl", 100, true);

  pointcloud_sub_ =
      nh_private_.subscribe(nh_private.param<std::string>("scan_topic", "fail"),
                            10,
                            &MeshLocalizerRos::associatePointCloud,
                            this);
  model_.type = visualization_msgs::Marker::MESH_RESOURCE;
  model_.ns = "primitive_surfaces";
  if (!nh_private_.hasParam("stl_model"))
    std::cerr << "ERROR 'stl_model' not set as parameter." << std::endl;
  model_.mesh_resource =
      "file://" + nh_private_.param<std::string>("stl_model", "fail");
  model_.header.frame_id = map_frame_;
  model_.scale.x = 1.0;
  model_.scale.y = 1.0;
  model_.scale.z = 1.0;
  model_.pose.position.x = 0;
  model_.pose.position.y = 0;
  model_.pose.position.z = 0;
  model_.pose.orientation.x = 0.0;
  model_.pose.orientation.y = 0.0;
  model_.pose.orientation.z = 0.0;
  model_.pose.orientation.w = 1.0;
  model_.color.b = 0.0f;
  model_.color.g = 0.0f;
  model_.color.r = 1.0;
  model_.color.a = 1.0;

  transformSrv_ =
      nh.advertiseService("transformModel",
          &MeshLocalizerRos::transformModelCb, this);
}

MeshLocalizerRos::~MeshLocalizerRos() {}

void MeshLocalizerRos::associatePointCloud(const PointCloud &pc_msg) {
  SE3 initial(SE3::Position(0, 0, 0), SE3::Rotation(1, 0, 0, 0));
  mesh_localizer_.icm(pc_msg, initial);
//  Associations associations = mesh_localizer_.associatePointCloud(pc_msg);
//  CHECK_EQ(associations.points_from.cols(), pc_msg.width);
//  CHECK_EQ(associations.points_to.cols(), pc_msg.width);
//  CHECK_EQ(associations.distances.rows(), pc_msg.width);
//  visualization_msgs::Marker good_marker, bad_marker;
//  good_marker.header.frame_id = map_frame_;
//  good_marker.ns = "semantic_graph_matches";
//  good_marker.type = visualization_msgs::Marker::LINE_LIST;
//  good_marker.action = visualization_msgs::Marker::ADD;
//  good_marker.id = 0;
//
//  good_marker.scale.x = 0.01f;
//  good_marker.color.r = 0.0f;
//  good_marker.color.g = 1.0f;
//  good_marker.color.b = 0.0f;
//  good_marker.color.a = 0.7f;
//  bad_marker = good_marker;
//  bad_marker.color.r = 1.0f;
//  bad_marker.color.g = 0.0f;
//
//  geometry_msgs::Point p_from, p_to;
//  int differences = 0;
//  for (size_t i = 0u; i < pc_msg.width; ++i) {
//
//// todo: less copying
//    p_from.x = associations.points_from(0, i);
//    p_from.y = associations.points_from(1, i);
//    p_from.z = associations.points_from(2, i);
//    p_to.x = associations.points_to(0, i);
//    p_to.y = associations.points_to(1, i);
//    p_to.z = associations.points_to(2, i);
//    double length = associations.distances(i);
//    if (length > distance_threshold_) {
//      differences++;
//      bad_marker.color.r = 1.0f;
//      bad_marker.color.g = 0.0f;
//      bad_marker.points.push_back(p_from);
//      bad_marker.points.push_back(p_to);
//    } else {
//      good_marker.color.r = 0.0f;
//      good_marker.color.g = 1.0f;
//      good_marker.points.push_back(p_from);
//      good_marker.points.push_back(p_to);
//    }
//  }
//  std::cout << "differences " << differences << "/" << pc_msg.width
//            << std::endl;
//
//  good_matches_pub_.publish(good_marker);
//  bad_matches_pub_.publish(bad_marker);
//  model_pub_.publish(model_);
//
//  publishArchitectModel();
//
//  // todo: Filter out points that are close to several (non-parallel) planes.
}

bool MeshLocalizerRos::transformModelCb(std_srvs::Empty::Request &request,
                                        std_srvs::Empty::Response &response) {
  tf::StampedTransform transform;
  tf_listener_.lookupTransform(map_frame_, cad_frame_, ros::Time(0), transform);
  Eigen::Matrix3d rotation;
  tf::matrixTFToEigen(transform.getBasis(), rotation);
  Eigen::Vector3d translation;
  tf::vectorTFToEigen(transform.getOrigin(), translation);
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
  transformation.block(0, 0, 3, 3) = rotation;
  transformation.block(0, 3, 3, 1) = translation;
  mesh_localizer_.transformModel(transformation);
  return true;
}

void MeshLocalizerRos::publishArchitectModel() const {
//  PointCloud pc_msg = mesh_localizer_.getModelAsPointCloud();
//  pc_msg.header.frame_id = map_frame_;
//  pcl_conversions::toPCL(ros::Time(0), pc_msg.header.stamp);
//  arch_pub_.publish(pc_msg);
}
}
}
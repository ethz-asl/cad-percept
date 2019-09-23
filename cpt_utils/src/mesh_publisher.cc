#include <CGAL/IO/Polyhedron_iostream.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_utils/mesh_publisher.h>

namespace cad_percept {
namespace cpt_utils {

MeshPublisher::MeshPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Settings only needed here.
  bool publish_on_start_ = nh_private_.param<bool>("publish_on_start", true);
  bool latch_topic_ = nh_private_.param<bool>("latch_topic", true);

  // Set-up of node.
  pub_mesh_ = nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("mesh_out", 1, latch_topic_);
  publish_service_ =
      nh_private_.advertiseService("publish", &MeshPublisher::triggerPublishMesh, this);
  id_service_ =
      nh_private_.advertiseService("get_triangle_id", &MeshPublisher::getClosestTriangle, this);

  default_filename_ = nh_private_.param<std::string>("default_filename", "mesh.off");
  frame_name_ = nh_private_.param<std::string>("frame_name", "world");

  if (publish_on_start_) {
    if (publishOffFile()) {
      ROS_INFO_STREAM("Published Mesh on Start");
    } else {
      ROS_WARN_STREAM("Could not publish Mesh on Start");
    }
  }
}

bool MeshPublisher::publishOffFile(std::string filename) {
  // default parameter => replace by default filename.
  if (filename.empty()) {
    filename = default_filename_;
  }

  // Create mesh model.
  if (!cgal::MeshModel::create(filename, &mesh_model_)) {
    return false;
  }

  // Read Off file into polyhedron type
  cgal::Polyhedron mesh = mesh_model_->getMesh();

  // publish as mesh msg
  cgal_msgs::TriangleMeshStamped mesh_msg;
  mesh_msg.header.stamp = ros::Time::now();
  mesh_msg.header.frame_id = frame_name_;
  cgal::triangleMeshToMsg(mesh, &mesh_msg.mesh);
  pub_mesh_.publish(mesh_msg);
  return true;
}

bool MeshPublisher::triggerPublishMesh(cgal_msgs::PublishMesh::Request &req,
                                       cgal_msgs::PublishMesh::Response &res) {
  bool result = publishOffFile(req.path);
  res.success = result;
  res.message = result ? "Published Mesh" : "Mesh not published";
  return true;
}

bool MeshPublisher::getClosestTriangle(cgal_msgs::FacetID::Request &req,
                                       cgal_msgs::FacetID::Response &res) {
  if (mesh_model_ == nullptr) return false;
  cgal::PointAndPrimitiveId ppid = mesh_model_->getClosestTriangle(
      (double)req.point.x, (double)req.point.y, (double)req.point.z);
  std::string facet_id = mesh_model_->getIdFromFacetHandle(ppid.second);
  res.facet_id = facet_id;
  return true;
}

}  // namespace cpt_utils
}  // namespace cad_percept

#include <CGAL/IO/Polyhedron_iostream.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cpt_utils/mesh_publisher.h>

namespace cad_percept {
namespace cpt_utils {

MeshPublisher::MeshPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private) {
  pub_mesh_ = nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("mesh_out", 1, true);
  publish_service_ = nh_private_.advertiseService("publish", &MeshPublisher::triggerService, this);
  default_filename_ = nh_private_.param<std::string>("default_filename", "mesh.off");
  frame_name_ = nh_private_.param<std::string>("frame_name", "world");
}

bool MeshPublisher::publishOffFile(const std::string filename) {
  std::ifstream infile(filename.c_str());

  // Check if file is readable.
  if (!infile.good()) {
    return false;
  }

  // Read Off file into polyhedron type
  cgal::Polyhedron mesh;
  infile >> mesh;  // such a nice operator overload!

  // publish as mesh msg
  cgal_msgs::TriangleMeshStamped mesh_msg;
  mesh_msg.header.stamp = ros::Time::now();
  mesh_msg.header.frame_id = frame_name_;
  cgal::triangleMeshToMsg(mesh, &mesh_msg.mesh);
  pub_mesh_.publish(mesh_msg);
  return true;
}

bool MeshPublisher::triggerService(std_srvs::Trigger::Request &req,
                                   std_srvs::Trigger::Response &res) {
  bool result = publishOffFile(default_filename_);
  res.success = result;
  res.message = result ? "Published and Latched Mesh" : "Mesh not published";
  return true;
}
}  // namespace cpt_utils
}  // namespace cad_percept

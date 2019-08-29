#include <cgal_conversions/mesh_conversions.h>
#include <cpt_utils/mesh_publisher.h>

namespace cad_percept {
namespace cpt_utils {
bool MeshPublisher::publishOffFile(const std::string filename) {
  // Read Off file into polyhedron type
  cgal::Polyhedron mesh;

  // publish as mesh msg
  cgal_msgs::TriangleMesh mesh_msg;
  cgal::triangleMeshToMsg(mesh, &mesh_msg);
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
}
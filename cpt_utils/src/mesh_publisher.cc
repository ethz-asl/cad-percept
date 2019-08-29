#include <cgal_conversions/mesh_conversions.h>
#include <cpt_utils/mesh_publisher.h>

namespace cad_percept {
namespace cpt_utils {
void MeshPublisher::publishOffFile(const std::string filename) {
  // Read Off file into polyhedron type
  cgal::Polyhedron mesh;

  // publish as mesh msg
  cgal_msgs::TriangleMesh mesh_msg;
  cgal::triangleMeshToMsg(mesh, &mesh_msg);
  pub_mesh_.publish(mesh_msg);
}
}  // namespace cpt_utils
}
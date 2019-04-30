#include <cgal_conversions/mesh_conversions.h>

namespace cad_percept {
namespace cgal {

void meshToVerticePointCloud(const Polyhedron &mesh, PointCloud *pc) {
  pc->width = mesh.size_of_vertices();
  pc->header.frame_id = "mesh";
  pcl::PointXYZ point;
  for (auto vertex_point = mesh.points_begin();
       vertex_point != mesh.points_end(); ++vertex_point) {
    point.x = vertex_point->x();
    point.y = vertex_point->y();
    point.z = vertex_point->z();
    pc->push_back(point);
  }
}
}
}

# include <cgal_interface/cgal_conversions.h>

namespace cad_percept {
namespace cgal {

void meshToPointCloud(const SurfaceMesh &mesh,
                      PointCloud *pc) {
  pc->width = mesh.size_of_vertices();
  pc->header.frame_id = "mesh";
  pcl::PointXYZ point;
  for (auto vertex = mesh.points_begin();
       vertex != mesh.points_end(); ++vertex) {
    point.x = vertex->x();
    point.y = vertex->y();
    point.z = vertex->z();
    pc->push_back(point);
  }
}
}
}
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cpt_collision_manifolds/offset_surface/multiple_vertex_normal_strategy.h>
#include <Eigen/Dense>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

bool MultipleVertexNormalStrategy::execute(const cad_percept::cgal::Polyhedron& surface,
                                           double offset,
                                           cad_percept::cgal::Polyhedron* offset_surface) {
  surface_ = surface;
  getFaceNormals();

  // calculate all face normals with new method
  for (auto vertex = surface_.vertices_begin(); vertex != surface_.vertices_end(); ++vertex) {
    std::pair<cgal::vertex_descriptor, cgal::Vector> normal_pair;
    normal_pair.first = vertex;
    createNewVertex(vertex, &normal_pair.second);
    vnormals_.insert(normal_pair);
  }

  // use simple vertex movement as a test.
  std::for_each(
      vnormals_.begin(), vnormals_.end(),
      std::bind(&MultipleVertexNormalStrategy::moveVertex, this, std::placeholders::_1, offset));

  *offset_surface = surface_;
}

void MultipleVertexNormalStrategy::getFaceNormals() {
  boost::associative_property_map<FaceNormalMap> map_fnormals(fnormals_);
  CGAL::Polygon_mesh_processing::compute_face_normals(surface_, map_fnormals);
}

bool MultipleVertexNormalStrategy::createNewVertex(cgal::vertex_descriptor& vertex,
                                                   cgal::Vector* normal) {
  *normal = cgal::Vector(0, 0, 0);
  uint count_faces = 0;

  // iterate through all incident faces
  for (const cgal::halfedge_descriptor& d : CGAL::halfedges_around_source(vertex, surface_)) {
    cgal::Vector face_normal = fnormals_[d->face()];
    *normal += face_normal;
    count_faces++;
  }

  // normalize
  if (count_faces != 0) {
    *normal /= count_faces;
  }
  return true;
}

void MultipleVertexNormalStrategy::moveVertex(
    std::pair<cgal::vertex_descriptor, cgal::Vector> vertex, const double offset) const {
  const cgal::Vector displacement = vertex.second * offset;
  vertex.first->point() += displacement;  // In place change of vertex
}

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

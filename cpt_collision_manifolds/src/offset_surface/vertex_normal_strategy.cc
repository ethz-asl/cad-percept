#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cpt_collision_manifolds/offset_surface/vertex_normal_strategy.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

bool VertexNormalStrategy::execute(const cad_percept::cgal::Polyhedron& surface,
                                   const double offset,
                                   cad_percept::cgal::Polyhedron* offset_surface) {
  // TODO(mpantic): Check performance / copy semantics of doing this.
  *offset_surface = surface;

  // Compute normals and store them in a map
  std::map<cgal::vertex_descriptor, cgal::Vector> vnormals;
  boost::associative_property_map<std::map<cgal::vertex_descriptor, cgal::Vector>> map_vnormals(
      vnormals);
  CGAL::Polygon_mesh_processing::compute_vertex_normals(*offset_surface, map_vnormals);

  // move Vertex along normal for each vertex-normal pair
  // could be easily parallelized using parallel_for_each or similar.
  std::for_each(vnormals.begin(), vnormals.end(),
                std::bind(&VertexNormalStrategy::moveVertex, this, std::placeholders::_1, offset));

  return true;
}

void VertexNormalStrategy::moveVertex(std::pair<cgal::vertex_descriptor, cgal::Vector> vertex,
                                      const double offset) const {
  const cgal::Vector displacement = vertex.second * offset;
  vertex.first->point() += displacement;  // In place change of vertex
}

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

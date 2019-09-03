#include <cpt_collision_manifolds/offset_surface/multiple_vertex_normal_strategy.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

bool MultipleVertexNormalStrategy::execute(const cad_percept::cgal::Polyhedron& surface,
                                           double offset,
                                           cad_percept::cgal::Polyhedron* offset_surface) {
  return false;
}

void MultipleVertexNormalStrategy::getFaceNormals(const cad_percept::cgal::Polyhedron& surface,
                                                  FaceNormalMap* fnormals) {}

bool MultipleVertexNormalStrategy::createNewVertex(cgal::vertex_descriptor vertex) { return false; }

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

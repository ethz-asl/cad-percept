#include <cpt_collision_manifolds/offset_surface/vertex_normal_strategy.h>
namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

bool VertexNormalStrategy::execute(const cad_percept::cgal::Polyhedron& surface,
                                   const double offset,
                                   cad_percept::cgal::Polyhedron* offset_surface) {
  std::cout << "test" << std::endl;
  return true;
}

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

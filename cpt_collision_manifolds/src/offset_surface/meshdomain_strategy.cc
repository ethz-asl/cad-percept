#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cpt_collision_manifolds/offset_surface/meshdomain_strategy.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

bool MeshDomainStrategy::execute(const cad_percept::cgal::Polyhedron& surface, const double offset,
                                 cad_percept::cgal::Polyhedron* offset_surface) {
  return false;
}
}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

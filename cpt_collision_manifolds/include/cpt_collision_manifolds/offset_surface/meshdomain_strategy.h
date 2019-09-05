#ifndef CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MESHDOMAIN_STRATEGY_H_
#define CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MESHDOMAIN_STRATEGY_H_
#include <cpt_collision_manifolds/offset_surface/construction_strategy.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

/*
 * Strategy based on CGAL's implicit mesh domain
 *
 */
class MeshDomainStrategy : public ConstructionStrategy {
 public:
  bool execute(const cad_percept::cgal::Polyhedron& surface, double offset,
               cad_percept::cgal::Polyhedron* offset_surface);
};

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept
#endif  // CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MESHDOMAIN_STRATEGY_H_

#ifndef CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_CONSTRUCTION_STRATEGY_H_
#define CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_CONSTRUCTION_STRATEGY_H_
#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

/*
 * Strategy pattern for the
 * offset surface construction Strategy
 */
class ConstructionStrategy {
 public:
  typedef std::shared_ptr<ConstructionStrategy> Ptr;

  virtual bool execute(const cad_percept::cgal::Polyhedron& surface, double offset,
                       cad_percept::cgal::Polyhedron* offset_surface) = 0;
};

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept
#endif  // CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_CONSTRUCTION_STRATEGY_H_

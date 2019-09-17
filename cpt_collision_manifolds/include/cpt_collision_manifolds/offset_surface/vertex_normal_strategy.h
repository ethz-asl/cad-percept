#ifndef CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_VERTEX_NORMAL_STRATEGY_H_
#define CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_VERTEX_NORMAL_STRATEGY_H_
#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_collision_manifolds/offset_surface/construction_strategy.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

/*
 * Strategy that moves the vertices along their computed normal
 * in order to construct the offset surface.
 *
 */
class VertexNormalStrategy : public ConstructionStrategy {
 public:
  explicit VertexNormalStrategy(ConfigProvider::Ptr cfg) : ConstructionStrategy(std::move(cfg)) {}

  bool execute(const cad_percept::cgal::Polyhedron& surface, double offset,
               cad_percept::cgal::Polyhedron* offset_surface);

 private:
  void moveVertex(std::pair<cgal::vertex_descriptor, cgal::Vector> vertex, double offset) const;
};

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept
#endif  // CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_VERTEX_NORMAL_STRATEGY_H_

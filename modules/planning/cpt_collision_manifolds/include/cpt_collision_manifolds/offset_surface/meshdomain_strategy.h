#ifndef CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MESHDOMAIN_STRATEGY_H_
#define CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MESHDOMAIN_STRATEGY_H_
#include <cgal_definitions/cgal_meshing_typedefs.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_collision_manifolds/offset_surface/construction_strategy.h>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

/*
 * Strategy based on CGAL's implicit mesh domain
 *
 */
class OffsetFunction {
 public:
  OffsetFunction(const cgal::Polyhedron& tm, const double offset_distance)
      : m_tree_ptr(
            new cgal::PolyhedronAABBTree(boost::begin(faces(tm)), boost::end(faces(tm)), tm)),
        m_side_of_ptr(new cgal::PolyhedronSideOf(*m_tree_ptr)),
        m_offset_distance(offset_distance),
        m_is_closed(is_closed(tm)) {
    CGAL_assertion(!m_tree_ptr->empty());
    m_tree_ptr->accelerate_distance_queries();
  }

  double operator()(const cgal::Point& p) const {
    using CGAL::sqrt;

    CGAL::Bounded_side side = m_is_closed ? m_side_of_ptr->operator()(p) : CGAL::ON_UNBOUNDED_SIDE;
    if (side == CGAL::ON_BOUNDARY) return m_offset_distance;

    cgal::Point closest_point = m_tree_ptr->closest_point(p);
    double distance = sqrt(squared_distance(p, closest_point));

    return (side == CGAL::ON_UNBOUNDED_SIDE ? -distance : distance) + m_offset_distance;
  }

 private:
  boost::shared_ptr<cgal::PolyhedronAABBTree> m_tree_ptr;
  boost::shared_ptr<cgal::PolyhedronSideOf> m_side_of_ptr;
  double m_offset_distance;
  bool m_is_closed;
};

class MeshDomainStrategy : public ConstructionStrategy {
 public:
  explicit MeshDomainStrategy(ConfigProvider::Ptr cfg) : ConstructionStrategy(std::move(cfg)) {}

  bool execute(const cad_percept::cgal::Polyhedron& surface, double offset,
               cad_percept::cgal::Polyhedron* offset_surface);
  // define mesh domain
};

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept
#endif  // CPT_COLLISION_MANIFOLDS_OFFSET_SURFACE_MESHDOMAIN_STRATEGY_H_

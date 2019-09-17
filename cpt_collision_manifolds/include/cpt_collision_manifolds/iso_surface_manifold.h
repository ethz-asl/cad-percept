#ifndef CPT_COLLISION_MANIFOLDS_ISOSURFACEMANIFOLD_H
#define CPT_COLLISION_MANIFOLDS_ISOSURFACEMANIFOLD_H
#include <cpt_collision_manifolds/collision_manifold_interface.h>
#include <cpt_collision_manifolds/offset_surface/vertex_normal_strategy.h>

namespace cad_percept {
namespace collision_manifolds {

/*
 * Simplest collision avoidance based on Manifolds.
 * Assumes body to be uniform shaped sphere.
 * Constructs Isosurface of the Mesh with the given radius.
 */
class IsoSurfaceManifold : public CollisionManifoldInterface {
 public:
  IsoSurfaceManifold(const cgal::PolyhedronPtr manifold, const double body_radius,
                     offset_surface::ConstructionStrategy::Ptr strategy)
      : body_radius_(body_radius),
        is_constructed_(false),
        original_manifold_(std::move(manifold)),
        offset_constructor_(std::move(strategy)) {}

  /*
   * Interface methods:
   */
  void setBodyAttitude(const Eigen::Quaterniond& attitude);
  double signedDistance(const Eigen::Vector3d& position);
  void getAsMesh(cgal::Polyhedron* mesh);
  void construct();

 private:
  // Private default constructor.
  IsoSurfaceManifold() : body_radius_(0.0) {}

  const double body_radius_;
  bool is_constructed_;
  const cgal::PolyhedronPtr original_manifold_;
  cgal::Polyhedron collision_manifold_;

  const offset_surface::ConstructionStrategy::Ptr offset_constructor_;
};
}  // namespace collision_manifolds
}  // namespace cad_percept

#endif  // CPT_COLLISION_MANIFOLDS_ISOSURFACEMANIFOLD_H

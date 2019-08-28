#ifndef CPT_COLLISION_MANIFOLDS_ISOSURFACEMANIFOLD_H
#define CPT_COLLISION_MANIFOLDS_ISOSURFACEMANIFOLD_H
#include <cpt_collision_manifolds/collision_manifold_interface.h>

namespace cad_percept {
namespace cpt_collision_manifolds {

/*
 * Simplest collision avoidance based on Manifolds.
 * Assumes body to be uniform shaped sphere.
 * Constructs Isosurface of the Mesh with the given radius.
 */
class IsoSurfaceManifold : CollisionManifoldInterface {
 public:
  IsoSurfaceManifold(double body_radius) :
      body_radius_(body_radius),
      is_constructed_(false) {}

  /*
   * Interface methods:
   */
  void setBodyAttitude(const Eigen::Quaterniond& attitude);
  double signedDistance(const Eigen::Vector3d& position);
  void getAsMesh(cgal::PolyhedronPtr mesh);
  void construct();

 private:
  // Private default constructor.
  IsoSurfaceManifold() : body_radius_(0.0) {}

  const double body_radius_;
  bool is_constructed_;

};
}
}

#endif // CPT_COLLISION_MANIFOLDS_ISOSURFACEMANIFOLD_H

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
      body_radius_(body_radius) {}

  /*
   * Interface methods:
   */

  void setBodyAttitude(const Eigen::Quaterniond& attitude);
  double signedDistance(const Eigen::Vector3d& position);
  void getAsMesh(cgal::PolyhedronPtr mesh);

 private:
  // Private default constructor.
  IsoSurfaceManifold() : body_radius_(0.0) {}

  const double body_radius_;

};
}
}

#endif // CPT_COLLISION_MANIFOLDS_ISOSURFACEMANIFOLD_H

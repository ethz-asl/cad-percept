#ifndef CPT_COLLISION_MANIFOLDS_COLLISION_MANIFOLD_INTERFACE_H
#define CPT_COLLISION_MANIFOLDS_COLLISION_MANIFOLD_INTERFACE_H

#include <Eigen/Dense>
#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace cpt_collision_manifolds {

class CollisionManifoldInterface {
 public:
  /*
   * Sets the current Body's orientation in 3D Space
   */
  virtual void setBodyAttitude(const Eigen::Quaterniond& pose) = 0;

  /*
   * Returns a signed distance to the manifold based on the Body position.
   */
  virtual double signedDistance(const Eigen::Vector3d& position) = 0;

  /*
   * Constructs the collision manifold as a mesh,
   * based on the current body attitude.
   */
  virtual void getAsMesh(cgal::PolyhedronPtr mesh) = 0;

};
}
}
#endif //CPT_COLLISION_MANIFOLDS_COLLISION_MANIFOLD_INTERFACE_H

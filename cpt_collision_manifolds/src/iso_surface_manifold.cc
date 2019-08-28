#include <cpt_collision_manifolds/iso_surface_manifold.h>

namespace cad_percept {
namespace cpt_collision_manifolds {

void IsoSurfaceManifold::construct() {}

void IsoSurfaceManifold::setBodyAttitude(const Eigen::Quaterniond& attitude) {
  // We don't need the attitude, so nothing is happening here.
}

double IsoSurfaceManifold::signedDistance(const Eigen::Vector3d& position) { return 0.0; }

void IsoSurfaceManifold::getAsMesh(cad_percept::cgal::PolyhedronPtr mesh) {}

}  // namespace cpt_collision_manifolds
}
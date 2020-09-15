#include <cgal_definitions/not_implemented_exception.h>
#include <cpt_collision_manifolds/iso_surface_manifold.h>

namespace cad_percept {
namespace collision_manifolds {

void IsoSurfaceManifold::construct() {
  bool success =
      offset_constructor_->execute(*original_manifold_, body_radius_, &collision_manifold_);

  is_constructed_ = success;
}

void IsoSurfaceManifold::setBodyAttitude(const Eigen::Quaterniond& attitude) {
  // We don't need the attitude, so nothing is happening here.
}

double IsoSurfaceManifold::signedDistance(const Eigen::Vector3d& position) {
  throw cad_percept::NotImplementedException();
}

void IsoSurfaceManifold::getAsMesh(cad_percept::cgal::Polyhedron* mesh) {
  // Create a copy
  *mesh = cgal::Polyhedron(collision_manifold_);
}

}  // namespace collision_manifolds
}  // namespace cad_percept

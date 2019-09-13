#include <CGAL/squared_distance_3.h>
#include <cgal_conversions/eigen_conversions.h>
#include <cpt_collision_manifolds/iso_surface_manifold.h>

namespace cad_percept {
namespace collision_manifolds {

void IsoSurfaceManifold::construct() {
  cgal::Polyhedron collision_poly;
  bool success = offset_constructor_->execute(*original_manifold_, body_radius_, &collision_poly);

  // non-explicit constructor of MeshModel takes care of this conversion.
  *collision_manifold_ = collision_poly;

  is_constructed_ = success;
}

void IsoSurfaceManifold::setBodyAttitude(const Eigen::Quaterniond& attitude) {
  // We don't need the attitude, so nothing is happening here.
}

double IsoSurfaceManifold::signedDistance(const Eigen::Vector3d& position) {
  cgal::Point position_cgal = cgal::eigenVectorToCgalPoint(position);
  cgal::PointAndPrimitiveId pt = collision_manifold_->getClosestTriangle(position_cgal);
  return sqrt(CGAL::squared_distance(position_cgal, pt.first));
}

void IsoSurfaceManifold::getAsMesh(cad_percept::cgal::Polyhedron* mesh) {
  // Create a copy
  *mesh = cgal::Polyhedron(collision_manifold_->getMesh());
}

}  // namespace collision_manifolds
}
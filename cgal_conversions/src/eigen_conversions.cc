#include <cgal_conversions/eigen_conversions.h>

namespace cad_percept {
namespace cgal {

void cgalPointToEigenVector(const Point *point, Eigen::Vector3d *vector) {
  (*vector)(0) = point->x();
  (*vector)(1) = point->y();
  (*vector)(2) = point->z();
}

void eigenVectorToCgalPoint(const Eigen::Vector3d *vector, Point *point) {
  *point = Point((*vector)(0), (*vector)(1), (*vector)(2));
}

Eigen::Vector3d cgalPointToEigenVector(const Point &point) {
  return Eigen::Vector3d(point.x(), point.y(), point.z());
}

Point eigenVectorToCgalPoint(const Eigen::Vector3d &vector) {
  return Point(vector(0), vector(1),vector(2));
}

Eigen::Vector3d vectorToEigenVector(const cgal::Vector &v) {
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

Vector eigenVectorToVector(const Eigen::Vector3d &ve) {
  return cgal::Vector(ve(0,0), ve(1,0), ve(2,0));
}

void vectorToEigenVector(const Vector *cvector, Eigen::Vector3d *vector) {
  (*vector)(0) = cvector->x();
  (*vector)(1) = cvector->y();
  (*vector)(2) = cvector->z();
}

void eigenVectorToVector(const Eigen::Vector3d *vector, Vector *cvector) {
  *cvector = Vector((*vector)(0), (*vector)(1), (*vector)(2));
}

}
}
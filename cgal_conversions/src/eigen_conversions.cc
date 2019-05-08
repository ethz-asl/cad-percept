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

void cgalVectorToEigenVector(const Vector *cvector, Eigen::Vector3d *vector) {
  (*vector)(0) = cvector->x();
  (*vector)(1) = cvector->y();
  (*vector)(2) = cvector->z();
}

void eigenVectorToCgalVector(const Eigen::Vector3d *vector, Vector *cvector) {
  *cvector = Vector((*vector)(0), (*vector)(1), (*vector)(2));
}

Eigen::Vector3d cgalVectorToEigenVector(const cgal::Vector &cvector) {
  return Eigen::Vector3d(cvector.x(), cvector.y(), cvector.z());
}

Vector eigenVectorToCgalVector(const Eigen::Vector3d &vector) {
  return cgal::Vector(vector(0,0), vector(1,0), vector(2,0));
}

}
}
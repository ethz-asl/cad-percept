#include <cgal_conversions/mesh_conversions.h>

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

}
}
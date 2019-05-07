#include <cgal_conversions/eigen_conversions.h>

namespace cad_percept {
namespace cgal {


Eigen::Vector3d vectorToEigenVector(const cgal::Vector &v) {
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

Vector eigenVectorToVector(const Eigen::Vector3d &ve) {
  return cgal::Vector(ve(0,0), ve(1,0), ve(2,0));
}

}
}
#ifndef EIGEN_CONVERSIONS_H
#define EIGEN_CONVERSIONS_H

#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace cgal {

Eigen::Vector3d vectorToEigenVector(const cgal::Vector &v);
Vector eigenVectorToVector(const Eigen::Vector3d &ve);

}
}

#endif // EIGEN_CONVERSIONS_H
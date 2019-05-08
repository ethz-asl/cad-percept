#ifndef CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H
#define CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H

#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace cgal {

void cgalPointToEigenVector(const Point *point, Eigen::Vector3d *vector);
void eigenVectorToCgalPoint(const Eigen::Vector3d *vector, Point *point);
Eigen::Vector3d cgalPointToEigenVector(const Point &point);
Point eigenVectorToCgalPoint(const Eigen::Vector3d &vector);
void vectorToEigenVector(const Vector *cvector, Eigen::Vector3d *vector);
void eigenVectorToVector(const Eigen::Vector3d *vector, Vector *cvector);
Eigen::Vector3d vectorToEigenVector(const Vector &v);
Vector eigenVectorToVector(const Eigen::Vector3d &ve);

}
}

#endif  // CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H

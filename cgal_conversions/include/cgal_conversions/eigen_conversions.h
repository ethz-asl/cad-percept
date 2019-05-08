#ifndef CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H
#define CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H

#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace cgal {

void cgalPointToEigenVector(const Point *point, Eigen::Vector3d *vector);
void eigenVectorToCgalPoint(const Eigen::Vector3d *vector, Point *point);

}
}

#endif  // CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H

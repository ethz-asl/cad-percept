#ifndef CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H
#define CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H

#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace cgal {

// Point <-> Vector
void cgalPointToEigenVector(const Point &point, Eigen::Vector3d *vector);
void eigenVectorToCgalPoint(const Eigen::Vector3d &vector, Point *point);
Eigen::Vector3d cgalPointToEigenVector(const Point &point);
Point eigenVectorToCgalPoint(const Eigen::Vector3d &vector);

// Vector <-> Vector
void cgalVectorToEigenVector(const Vector &cvector, Eigen::Vector3d *vector);
void eigenVectorToCgalVector(const Eigen::Vector3d &vector, Vector *cvector);
Eigen::Vector3d cgalVectorToEigenVector(const Vector &cvector);
Vector eigenVectorToCgalVector(const Eigen::Vector3d &vector);

// Transformation <-> Transformation
void cgalTransformationToEigenTransformation(const Transformation &ctransformation,
                                             Eigen::Matrix4d *transformation);
void eigenTransformationToCgalTransformation(const Eigen::Matrix4d &transformation,
                                             Transformation *ctransformation);
Eigen::Matrix4d cgalTransformationToEigenTransformation(const Transformation &ctransformation);
Transformation eigenTransformationToCgalTransformation(const Eigen::Matrix4d &transformation);
}  // namespace cgal
}  // namespace cad_percept

#endif  // CGAL_CONVERSIONS_EIGEN_CONVERSIONS_H

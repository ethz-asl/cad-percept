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

Eigen::Vector3d cgalVectorToEigenVector(const Vector &cvector) {
  return Eigen::Vector3d(cvector.x(), cvector.y(), cvector.z());
}

Vector eigenVectorToCgalVector(const Eigen::Vector3d &vector) {
  return Vector(vector(0,0), vector(1,0), vector(2,0));
}

void cgalTransformationToEigenTransformation(const Transformation *ctransformation, Eigen::Matrix4d *transformation) {
  if (ctransformation->m(3,3) != 1) {
    std::cerr << "Transformation Matrix is not an affine transformation with m(3,3) = 1!" << std::endl;
  }

                      (*transformation)(0,0) = ctransformation->m(0,0);
                      (*transformation)(0,1) = ctransformation->m(0,1);
                      (*transformation)(0,2) = ctransformation->m(0,2);
                      (*transformation)(0,3) = ctransformation->m(0,3);
                      (*transformation)(1,0) = ctransformation->m(1,0);
                      (*transformation)(1,1) = ctransformation->m(1,1);
                      (*transformation)(1,2) = ctransformation->m(1,2);
                      (*transformation)(1,3) = ctransformation->m(1,3);
                      (*transformation)(2,0) = ctransformation->m(2,0);
                      (*transformation)(2,1) = ctransformation->m(2,1);
                      (*transformation)(2,2) = ctransformation->m(2,2);
                      (*transformation)(2,3) = ctransformation->m(2,3);
                      (*transformation)(3,0) = 0.0;
                      (*transformation)(3,1) = 0.0;
                      (*transformation)(3,2) = 0.0;
                      (*transformation)(3,3) = ctransformation->m(3,3);
}

void eigenTransformationToCgalTransformation(const Eigen::Matrix4d *transformation, Transformation *ctransformation) {
  if ((*transformation)(3,0) != 0 || (*transformation)(3,1) != 0 || (*transformation)(3,2) != 0 || (*transformation)(3,3) != 1) {
    std::cerr << "Transformation Matrix is not an affine transformation with m(3,3) = 1!" << std::endl;
  }

  *ctransformation = Transformation(
                      (*transformation)(0,0),
                      (*transformation)(0,1),
                      (*transformation)(0,2),
                      (*transformation)(0,3),
                      (*transformation)(1,0),
                      (*transformation)(1,1),
                      (*transformation)(1,2),
                      (*transformation)(1,3),
                      (*transformation)(2,0),
                      (*transformation)(2,1),
                      (*transformation)(2,2),
                      (*transformation)(2,3),
                      (*transformation)(3,3));
}

Eigen::Matrix4d cgalTransformationToEigenTransformation(const Transformation &ctransformation) {
  if (ctransformation.m(3,3) != 1) {
    std::cerr << "Transformation Matrix is not an affine transformation with m(3,3) = 1!" << std::endl;
  }

  Eigen::Matrix4d transformation;
  (transformation)(0,0) = ctransformation.m(0,0);
  (transformation)(0,1) = ctransformation.m(0,1);
  (transformation)(0,2) = ctransformation.m(0,2);
  (transformation)(0,3) = ctransformation.m(0,3);
  (transformation)(1,0) = ctransformation.m(1,0);
  (transformation)(1,1) = ctransformation.m(1,1);
  (transformation)(1,2) = ctransformation.m(1,2);
  (transformation)(1,3) = ctransformation.m(1,3);
  (transformation)(2,0) = ctransformation.m(2,0);
  (transformation)(2,1) = ctransformation.m(2,1);
  (transformation)(2,2) = ctransformation.m(2,2);
  (transformation)(2,3) = ctransformation.m(2,3);
  (transformation)(3,0) = 0.0;
  (transformation)(3,1) = 0.0;
  (transformation)(3,2) = 0.0;
  (transformation)(3,3) = ctransformation.m(3,3);
  return transformation;
}

Transformation eigenTransformationToCgalTransformation(const Eigen::Matrix4d &transformation) {
  if ((transformation)(3,0) != 0 || (transformation)(3,1) != 0 || (transformation)(3,2) != 0 || (transformation)(3,3) != 1) {
    std::cerr << "Transformation Matrix is not an affine transformation with m(3,3) = 1!" << std::endl;
  }

  return Transformation(
                      (transformation)(0,0),
                      (transformation)(0,1),
                      (transformation)(0,2),
                      (transformation)(0,3),
                      (transformation)(1,0),
                      (transformation)(1,1),
                      (transformation)(1,2),
                      (transformation)(1,3),
                      (transformation)(2,0),
                      (transformation)(2,1),
                      (transformation)(2,2),
                      (transformation)(2,3),
                      (transformation)(3,3));
}

}
}

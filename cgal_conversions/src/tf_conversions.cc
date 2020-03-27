#include <cgal_conversions/tf_conversions.h>

namespace cad_percept {
namespace cgal {

void tfTransformationToCGALTransformation(tf::StampedTransform transform,
                                          cad_percept::cgal::Transformation& ctransformation) {
  Eigen::Matrix3d rotation;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      rotation(i, j) = transform.getBasis()[i][j];
    }
  }
  Eigen::Vector3d translation;
  translation(0) = transform.getOrigin()[0];
  translation(1) = transform.getOrigin()[1];
  translation(2) = transform.getOrigin()[2];
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
  transformation.block(0, 0, 3, 3) = rotation;
  transformation.block(0, 3, 3, 1) = translation;
  cad_percept::cgal::eigenTransformationToCgalTransformation(transformation, &ctransformation);
}

}  // namespace cgal
}  // namespace cad_percept

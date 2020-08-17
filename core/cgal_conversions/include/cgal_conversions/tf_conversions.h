#ifndef CGAL_CONVERSIONS_TF_CONVERSIONS_H
#define CGAL_CONVERSIONS_TF_CONVERSIONS_H

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <tf_conversions/tf_eigen.h>

namespace cad_percept {
namespace cgal {

void tfTransformationToCGALTransformation(tf::StampedTransform transform,
                                          cad_percept::cgal::Transformation& ctransformation);
}  // namespace cgal
}  // namespace cad_percept

#endif

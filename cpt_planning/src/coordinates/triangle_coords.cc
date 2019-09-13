#include <cpt_planning/coordinates/triangle_coords.h>

namespace cad_percept {
namespace planning {

template <int N>
cgal::VectorReturn<N> TriangleCoords<N>::toCartesian(cgal::Vector3In& barycentric) const {
  return toCartesian(barycentric);
}

template <int N>
cgal::Vector3Return TriangleCoords<N>::toBarycentric(cgal::VectorIn<N>& point_on_triangle) const {
  return toBarycentric(point_on_triangle);
}

template <int N>
Eigen::Matrix<double, N, 1> TriangleCoords<N>::toCartesian(
    const Eigen::Vector3d& coordinates) const {
  return coordinates.x() * a1_ + coordinates.y() * a2_ + coordinates.z() * a3_;
}

template <int N>
template <int M>
cgal::VectorOut<M> TriangleCoords<N>::translateTo(const TriangleCoords<M>& other,
                                                  cgal::VectorIn<N>& point_on_triangle) const {
  Eigen::Vector3d barycentric = this->toBarycentric(point_on_triangle);
  return other->toCartesian(barycentric);
}

template <int N>
Eigen::Vector3d TriangleCoords<N>::toBarycentric(
    const Eigen::Matrix<double, N, 1>& point_on_triangle) const {
  Eigen::Matrix<double, N, 1> v0, v1, v2;  // intermediate values

  // works for 2d and 3d cartesian coordinates (barycentric coordinates are always 3d.)
  v0 = a2_ - a1_;
  v1 = a3_ - a1_;
  v2 = point_on_triangle - a1_;

  double d00 = v0.dot(v0);
  double d01 = v0.dot(v1);
  double d11 = v1.dot(v1);
  double d20 = v2.dot(v0);
  double d21 = v2.dot(v1);

  double denom = 1.0 / (d00 * d11 - d01 * d01);

  double v = (d11 * d20 - d01 * d21) * denom;
  double w = (d00 * d21 - d01 * d20) * denom;
  double u = 1.0 - v - w;
  return {u, v, w};
}

}  // namespace planning
}  // namespace cad_percept

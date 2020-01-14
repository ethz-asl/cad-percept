#include <cpt_planning/coordinates/triangle_coords.h>

namespace cad_percept {
namespace planning {

template <int N>
cgal::VectorReturn<N> TriangleCoords<N>::toCartesian(cgal::Vector3In barycentric) const {
  Eigen::Vector3d coordinates = barycentric;
  return static_cast<cgal::VectorReturn<N>>(coordinates.x() * a1_ + coordinates.y() * a2_ +
                                            coordinates.z() * a3_);
}

template <int N>
cgal::Vector3Return TriangleCoords<N>::toBarycentric(cgal::VectorIn<N> point_on_triangle) const {
  Eigen::Matrix<double, N, 1> v0, v1, v2;  // intermediate values

  // works for 2d and 3d cartesian coordinates (barycentric coordinates are always 3d.)
  v0 = a2_ - a1_;
  v1 = a3_ - a1_;
  v2 = static_cast<Eigen::Matrix<double, N, 1>>(point_on_triangle) - a1_;

  double d00 = v0.dot(v0);
  double d01 = v0.dot(v1);
  double d11 = v1.dot(v1);
  double d20 = v2.dot(v0);
  double d21 = v2.dot(v1);

  double denom = 1.0 / (d00 * d11 - d01 * d01);

  double v = (d11 * d20 - d01 * d21) * denom;
  double w = (d00 * d21 - d01 * d20) * denom;
  double u = 1.0 - v - w;
  return Eigen::Vector3d({u, v, w});
}

template <int N>
template <int M>
cgal::VectorReturn<M> TriangleCoords<N>::translateTo(const TriangleCoords<M>& other,
                                                     cgal::VectorIn<N> point_on_triangle) const {
  Eigen::Vector3d barycentric = this->toBarycentric(point_on_triangle);
  return other.toCartesian(barycentric);
}

// Explicit template instantiation so that all the methods are built.
template cgal::VectorReturn<2> TriangleCoords<2>::toCartesian(cgal::Vector3In barycentric) const;
template cgal::VectorReturn<3> TriangleCoords<3>::toCartesian(cgal::Vector3In barycentric) const;
template cgal::Vector3Return TriangleCoords<2>::toBarycentric(
    cgal::VectorIn<2> point_on_triangle) const;
template cgal::Vector3Return TriangleCoords<3>::toBarycentric(
    cgal::VectorIn<3> point_on_triangle) const;

template cgal::VectorReturn<3> TriangleCoords<2>::translateTo(
    const TriangleCoords<3>& other, cgal::VectorIn<2> point_on_triangle) const;

template cgal::VectorReturn<2> TriangleCoords<3>::translateTo(
    const TriangleCoords<2>& other, cgal::VectorIn<3> point_on_triangle) const;

}  // namespace planning
}  // namespace cad_percept

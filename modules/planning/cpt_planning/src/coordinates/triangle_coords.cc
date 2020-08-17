#include <cpt_planning/coordinates/triangle_coords.h>

namespace cad_percept {
namespace planning {

template<int N>
bool TriangleCoords<N>::isInside(cgal::VectorIn<N> point) const {
  Eigen::Vector3d barycentric = toBarycentric(point);
  return (barycentric.array() > 0.0 && barycentric.array() < 1.0).all();
}

template<int N>
cgal::VectorReturn<N> TriangleCoords<N>::toCartesian(cgal::Vector3In barycentric) const {
  Eigen::Vector3d coordinates = barycentric;
  return static_cast<cgal::VectorReturn<N>>(coordinates.x() * a1_
      + coordinates.y() * a2_ +
      coordinates.z() * a3_);
}

template<int N>
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

template<int N>
template<int M>
cgal::VectorReturn<M> TriangleCoords<N>::translateTo(const TriangleCoords<M> &other,
                                                     cgal::VectorIn<N> point_on_triangle) const {
  Eigen::Vector3d barycentric = this->toBarycentric(point_on_triangle);
  return other.toCartesian(barycentric);
}

template<int N>
Eigen::Matrix<double, N, 1> TriangleCoords<N>::getNormal() const {
  // Will cause a compilation errror for 2D triangle coords, as eigen's cross is not defined.
  static_assert(N != 2, "Cross product not implemented in 2D!");
  return (a2_ - a1_).cross(a3_ - a1_).normalized();
}

template<int N>
Eigen::Matrix<double, N, 1> TriangleCoords<N>::getVertex(uint id) const {
  switch (id % 3) {
    case 0:return a1_;
    case 1:return a2_;
    case 2:return a3_;
  }
}

template<int N>
template<int M>
Eigen::Matrix<double, N, N> TriangleCoords<N>::getJacobianWrt(const
                                                              TriangleCoords<M> &other) const {
  static_assert(N == 3 && M == 2, "Currently only implemented for N=3, M=2");
  Eigen::Vector3d A_xyz, B_xyz, C_xyz;  // triangle vertices in xyz
  Eigen::Vector3d A_uv, B_uv, C_uv;     // triangle vertices in uv

  A_xyz = this->getVertex(0);
  B_xyz = this->getVertex(1);
  C_xyz = this->getVertex(2);

  A_uv.topRows<2>() = other.getVertex(0);
  B_uv.topRows<2>() = other.getVertex(1);
  C_uv.topRows<2>() = other.getVertex(2);

  Eigen::Vector3d AB_xyz, AC_xyz, N_xyz;
  AB_xyz = B_xyz - A_xyz;  // "-A+B"
  AC_xyz = C_xyz - A_xyz;  // "-A+C"
  N_xyz = getNormal();

  Eigen::Vector3d I, K;  // vectors do build up the derivatve.
  Eigen::Vector3d part_u, part_v, part_h;
  double d00, d11, d01;
  double denom;

  // constants
  d00 = AB_xyz.dot(AB_xyz);
  d01 = AB_xyz.dot(AC_xyz);
  d11 = AC_xyz.dot(AC_xyz);
  denom = d00 * d11 - d01 * d01;

  I = (d00 * AC_xyz - d01 * AB_xyz) / denom;
  K = (d11 * AB_xyz - d01 * AC_xyz) / denom;

  // u/x, u/y, u/z
  part_u = B_uv[0] * K + C_uv[0] * I - A_uv[0] * (I + K);

  // v/x, v/y, v/z
  part_v = B_uv[1] * K + C_uv[1] * I - A_uv[1] * (I + K);

  // h/x, h/y, h/z
  part_h = N_xyz;

  /*        | u/x  u/y  u/z |
   * J_mtx =| v/x  v/y  v/z |
   *        | h/x  h/y  h/z |
   */
  Eigen::Matrix3d J_mtx;
  J_mtx.row(0) = part_u;
  J_mtx.row(1) = part_v;
  J_mtx.row(2) = part_h;

  return J_mtx;
}

// Explicit template instantiation so that all the methods are built.
template bool TriangleCoords<2>::isInside(cgal::VectorIn<2> point) const;
template bool TriangleCoords<3>::isInside(cgal::VectorIn<3> point) const;

template cgal::VectorReturn<2> TriangleCoords<2>::toCartesian(cgal::Vector3In barycentric) const;
template cgal::VectorReturn<3> TriangleCoords<3>::toCartesian(cgal::Vector3In barycentric) const;
template cgal::Vector3Return TriangleCoords<2>::toBarycentric(
    cgal::VectorIn<2> point_on_triangle) const;
template cgal::Vector3Return TriangleCoords<3>::toBarycentric(
    cgal::VectorIn<3> point_on_triangle) const;

template cgal::VectorReturn<3> TriangleCoords<2>::translateTo(
    const TriangleCoords<3> &other, cgal::VectorIn<2> point_on_triangle) const;

template cgal::VectorReturn<2> TriangleCoords<3>::translateTo(
    const TriangleCoords<2> &other, cgal::VectorIn<3> point_on_triangle) const;

template Eigen::Matrix<double, 3, 1> TriangleCoords<3>::getNormal() const;

template Eigen::Matrix<double,
                       2,
                       1> TriangleCoords<2>::getVertex(uint id) const;
template Eigen::Matrix<double,
                       3,
                       1> TriangleCoords<3>::getVertex(uint id) const;

template Eigen::Matrix<double, 3, 3> TriangleCoords<3>::getJacobianWrt(const
                                                                       TriangleCoords<
                                                                           2> &other) const;

}  // namespace planning
}  // namespace cad_percept

#include <cpt_planning/coordinates/barycentric.h>

namespace cad_percept {
namespace planning {

template <int N>
Barycentric<N>::Barycentric(cgal::Vector3In& values) {
  coordinates_ = values;
}

template <int N>
cgal::Vector3Return Barycentric<N>::asVector() {
  return coordinates_;
}

template <int N>
cgal::VectorReturn<N> Barycentric<N>::toCartesian(TupleTriangle<N>& triangle) {
  return toCartesian(std::get<0>(triangle), std::get<1>(triangle), std::get<2>(triangle));
}

template <int N>
void Barycentric<N>::fromCartesian(TupleTriangle<N>& triangle,
                                   cgal::VectorIn<N>& point_on_triangle) {
  coordinates_ = fromCartesian(std::get<0>(triangle), std::get<1>(triangle), std::get<2>(triangle),
                               point_on_triangle);
}

template <int N>
Eigen::Matrix<double, N, 1> Barycentric<N>::toCartesian(const Eigen::Matrix<double, N, 1>& a1,
                                                        const Eigen::Matrix<double, N, 1>& a2,
                                                        const Eigen::Matrix<double, N, 1>& a3) {
  return coordinates_.x() * a1 + coordinates_.y() * a2 + coordinates_.z() * a3;
}

template <int N>
Eigen::Vector3d Barycentric<N>::fromCartesian(
    const Eigen::Matrix<double, N, 1>& a1, const Eigen::Matrix<double, N, 1>& a2,
    const Eigen::Matrix<double, N, 1>& a3, const Eigen::Matrix<double, N, 1>& point_on_triangle) {
  Eigen::Matrix<double, N, 1> v0, v1, v2;  // intermediate values

  // works for 2d and 3d cartesian coordinates (barycentric coordinates are always 3d.)
  v0 = a2 - a1;
  v1 = a3 - a1;
  v2 = point_on_triangle - a1;

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

#ifndef CPT_PLANNING_COORDINATES_BARYCENTRIC_H_
#define CPT_PLANNING_COORDINATES_BARYCENTRIC_H_
#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>

namespace cad_percept {
namespace planning {

template <int N>
class TriangleCoords {
  static_assert(N == 2 || N == 3, "ONLY IMPLEMENTED FOR 2D/3D TYPES.");

 public:
  explicit TriangleCoords(cgal::Vector3In& a1, cgal::Vector3In& a2, cgal::Vector3In& a3)
      : a1_(a1), a2_(a2), a3_(a3) {}

  cgal::VectorReturn<N> toCartesian(cgal::Vector3In& barycentric);

  cgal::Vector3Return toBarycentric(cgal::VectorIn<N>& point_on_triangle);

 protected:
  explicit TriangleCoords(const Eigen::Matrix<double, N, 3>& a)
      : a1_(a.col(0)), a2_(a.col(1)), a3_(a.col(2)) {}

  Eigen::Matrix<double, N, 1> toCartesian(const Eigen::Vector3d& coordinates);

  Eigen::Vector3d toBarycentric(const Eigen::Matrix<double, N, 1>& point_on_triangle);

  const Eigen::Matrix<double, N, 1> a1_, a2_, a3_;
};


}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_COORDINATES_BARYCENTRIC_H_

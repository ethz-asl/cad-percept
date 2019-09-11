#ifndef CPT_PLANNING_COORDINATES_BARYCENTRIC_H_
#define CPT_PLANNING_COORDINATES_BARYCENTRIC_H_
#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>

namespace cad_percept {
namespace planning {

template <int N>
using TupleTriangle = std::tuple<cgal::VectorIn<N>, cgal::VectorIn<N>, cgal::VectorIn<N>>;

template <int N>
class Barycentric {
  static_assert(N == 2 || N == 3, "ONLY IMPLEMENTED FOR 2D/3D TYPES.");

 public:
  Barycentric(cgal::Vector3In& values);

  cgal::Vector3Return asVector();

  cgal::VectorReturn<N> toCartesian(TupleTriangle<N>& triangle);

  void fromCartesian(TupleTriangle<N>& triangle, cgal::VectorIn<N>& point_on_triangle);

 private:
  Eigen::Matrix<double, N, 1> toCartesian(const Eigen::Matrix<double, N, 1>& a1,
                                          const Eigen::Matrix<double, N, 1>& a2,
                                          const Eigen::Matrix<double, N, 1>& a3);

  Eigen::Vector3d fromCartesian(const Eigen::Matrix<double, N, 1>& a1,
                                const Eigen::Matrix<double, N, 1>& a2,
                                const Eigen::Matrix<double, N, 1>& a3,
                                const Eigen::Matrix<double, N, 1>& point_on_triangle);

  Eigen::Vector3d coordinates_;  // Actual coordinates
};
typedef Barycentric<2> Barycentric2d;
typedef Barycentric<3> Barycentric3d;

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_COORDINATES_BARYCENTRIC_H_

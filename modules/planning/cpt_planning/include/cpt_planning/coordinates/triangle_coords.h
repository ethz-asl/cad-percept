#ifndef CPT_PLANNING_COORDINATES_BARYCENTRIC_H_
#define CPT_PLANNING_COORDINATES_BARYCENTRIC_H_
#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_definitions/cgal_typedefs.h>

#include <Eigen/Dense>
#include <boost/optional.hpp>

namespace cad_percept {
namespace planning {

/*
 * Implements triangular barycentric coordinates and the conversions between barycentric and
 * cartesian.
 * Note that barycentric coordinates are always 3d, as they are a linear combination of the
 * vertices. That's why some vectors are templated on N (depending on the dimension in which the
 * triangle "lives" in), and some are always 3D (barycentrics).
 */
template <int N>
class TriangleCoords {
  static_assert(N == 2 || N == 3, "TRIANGLECOORDS ARE ONLY IMPLEMENTED FOR 2D/3D TYPES.");

 public:
  explicit TriangleCoords(cgal::VectorIn<N> a1, cgal::VectorIn<N>& a2, cgal::VectorIn<N>& a3)
      : a1_(a1.operator typename cgal::VectorAdap<N>::eigenVector()),
        a2_(a2.operator typename cgal::VectorAdap<N>::eigenVector()),
        a3_(a3.operator typename cgal::VectorAdap<N>::eigenVector()) {
    /*  Weird casts above.
     *  Reason: Eigen Changed the conversion constructors and this somehow
     *  breaks casting directly from our VectorAdapters (VectorIn<N>..)
     *  https://forum.kde.org/viewtopic.php?f=74&t=137768
     *  to be fixed later.
     *  Now this is an ugly hack to enforce casting.
     *  Even static_cast didn't work as it should?
     *
     */

  }

  cgal::VectorReturn<N> toCartesian(cgal::Vector3In barycentric) const;

  cgal::Vector3Return toBarycentric(cgal::VectorIn<N> point_on_triangle) const;

  /* Used to translate coordinates from one triangle to the other */
  template <int M>
  cgal::VectorReturn<M> translateTo(const TriangleCoords<M>& other,
                                    cgal::VectorIn<N> point_on_triangle) const;

  Eigen::Matrix<double, N, 1> getVertex(uint id) const;

  bool isInside(cgal::VectorIn<N> point) const;

  template <int M>
  Eigen::Matrix<double, N, N> getJacobianWrt(const TriangleCoords<M>& other) const;

  Eigen::Matrix<double, N, 1> getNormal() const;

 protected:
  explicit TriangleCoords(const Eigen::Matrix<double, N, 3>& a)
      : a1_(a.col(0)), a2_(a.col(1)), a3_(a.col(2)) {}

  const Eigen::Matrix<double, N, 1> a1_, a2_, a3_;  // base vectors of the triangle.
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_COORDINATES_BARYCENTRIC_H_

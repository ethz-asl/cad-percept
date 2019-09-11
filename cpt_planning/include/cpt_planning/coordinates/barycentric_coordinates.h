//
// Created by mpantic on 11.09.19.
//

#ifndef CPT_PLANNING_COORDINATES_BARYCENTRIC_COORDINATES_H_
#define CPT_PLANNING_COORDINATES_BARYCENTRIC_COORDINATES_H_
#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>

namespace cad_percept {
namespace planning {
typedef std::tuple<cgal::Vector3In, cgal::Vector3In, cgal::Vector3In> TupleTriangle;

class BarycentricCoordinates {
 public:
  BarycentricCoordinates(cgal::Vector3In& values);

  cgal::Vector3Return asVector();

  cgal::Vector3Return toCartesian(TupleTriangle& triangle);

  void fromCartesian(TupleTriangle& triangle, cgal::Vector3In& point_on_triangle);

 private:
  Eigen::Vector3d toCartesian(const Eigen::Vector3d& a1, const Eigen::Vector3d& a2,
                              const Eigen::Vector3d& a3);

  Eigen::Vector3d fromCartesian(const Eigen::Vector3d& a1, const Eigen::Vector3d& a2,
                                const Eigen::Vector3d& a3,
                                const Eigen::Vector3d& point_on_triangle);

  Eigen::Vector3d coordinates_;  // Actual coordinates
};

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_COORDINATES_BARYCENTRIC_COORDINATES_H_

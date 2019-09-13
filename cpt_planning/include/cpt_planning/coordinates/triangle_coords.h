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

template <int N>
class FaceCoords : TriangleCoords<N> {
 public:
  FaceCoords(cgal::face_descriptor face, const cgal::Polyhedron& mesh)
      : TriangleCoords<N>(FaceCoords<N>::staticInitializer(face, mesh)), face_(face) {}

 public:
  cgal::face_descriptor getFaceDescriptor() const { return face_; }

 private:
  static Eigen::Matrix<double, N, 3> staticInitializer(const cgal::face_descriptor face,
                                                       const cgal::Polyhedron& mesh) {
    int i = 0;
    Eigen::Matrix<double, N, 3> vertice_coords;

    for (const auto& halfedge : CGAL::halfedges_around_face(CGAL::halfedge(face, mesh), mesh)) {
      if (i >= 3) {
        break;
      }
      auto& vertex = halfedge->vertex()->point();
      vertice_coords.col(i)[0] = vertex.x();
      vertice_coords.col(i)[1] = vertex.y();
      if (N == 3) {
        vertice_coords.col(i)[2] = vertex.z();  // hack until we have templated meshmodel
      }
      ++i;
    }
    return vertice_coords;
  }

  const cgal::face_descriptor face_;
};
typedef FaceCoords<2> FaceCoords2d;
typedef FaceCoords<3> FaceCoords3d;

}  // namespace planning
}  // namespace cad_percept

#endif  // CPT_PLANNING_COORDINATES_BARYCENTRIC_H_

#ifndef CPT_PLANNING_COORDINATES_FACE_COORDS_H_
#define CPT_PLANNING_COORDINATES_FACE_COORDS_H_

#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_planning/coordinates/triangle_coords.h>
#include <Eigen/Dense>

namespace cad_percept {
namespace planning {

template <int N>
class FaceCoords : TriangleCoords<N> {
 public:
  inline FaceCoords(cgal::face_descriptor face, const cgal::Polyhedron& mesh)
      : TriangleCoords<N>(FaceCoords<N>::staticInitializer(face, mesh)), face_(face) {}

 public:
  inline cgal::face_descriptor getFaceDescriptor() const { return face_; }

 private:
  static Eigen::Matrix<double, N, 3> staticInitializer(const cgal::face_descriptor face,
                                                       const cgal::Polyhedron& mesh);
  const cgal::face_descriptor face_;
};
typedef FaceCoords<2> FaceCoords2d;
typedef FaceCoords<3> FaceCoords3d;

}  // namespace planning
}  // namespace cad_percept
#endif  // CPT_PLANNING_COORDINATES_FACE_COORDS_H_

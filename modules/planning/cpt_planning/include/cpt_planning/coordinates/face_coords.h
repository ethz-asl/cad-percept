#ifndef CPT_PLANNING_COORDINATES_FACE_COORDS_H_
#define CPT_PLANNING_COORDINATES_FACE_COORDS_H_

#include <cgal_conversions/cgal_eigen_adapter.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_planning/coordinates/triangle_coords.h>

#include <Eigen/Dense>

namespace cad_percept {
namespace planning {

/*
 * Class represents triangular barycentric coordinates mapped onto
 * a facet of a mesh.
 *
 * Are also an instance of regular TriangleCoordinates.
 */
template <int N>
class FaceCoords : public TriangleCoords<N> {
 private:
  // Needs to be implemented in header file, as it otherwise produces a linker error
  // (templated static functions can't be explicitly instantiated)
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

 public:
  inline FaceCoords(cgal::face_descriptor face, const cgal::Polyhedron& mesh)
      : TriangleCoords<N>(FaceCoords<N>::staticInitializer(face, mesh)), face_(face) {}

  inline cgal::face_descriptor getFaceDescriptor() const { return face_; }

  TriangleCoords<2> mapVertices(
      const CGAL::Unique_hash_map<cgal::vertex_descriptor, cgal::Point_2>& map) {
    // get 2d coordinates for each vertex.
    cgal::Polyhedron::Halfedge_around_facet_circulator it = face_->facet_begin();
    cgal::VectorIn<2> a = map[it++->vertex()];
    cgal::VectorIn<2> b = map[it++->vertex()];
    cgal::VectorIn<2> c = map[it++->vertex()];

    return TriangleCoords<2>(a, b, c);
  }

 private:
  const cgal::face_descriptor face_;
};
typedef FaceCoords<2> FaceCoords2d;
typedef FaceCoords<3> FaceCoords3d;

}  // namespace planning
}  // namespace cad_percept
#endif  // CPT_PLANNING_COORDINATES_FACE_COORDS_H_

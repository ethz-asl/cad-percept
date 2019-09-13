#include <cpt_planning/coordinates/face_coords.h>
namespace cad_percept {
namespace planning {

template <int N>
Eigen::Matrix<double, N, 3> FaceCoords<N>::staticInitializer(const cgal::face_descriptor face,
                                                             const cgal::PolyhedronPtr mesh) {
  int i = 0;
  Eigen::Matrix<double, N, 3> vertice_coords;

  for (const auto& halfedge : CGAL::halfedges_around_face(CGAL::halfedge(face, *mesh), *mesh)) {
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
}  // namespace planning
}  // namespace cad_percept

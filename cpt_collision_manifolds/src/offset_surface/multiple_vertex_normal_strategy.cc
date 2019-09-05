#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <cgal_conversions/eigen_conversions.h>
#include <cpt_collision_manifolds/offset_surface/multiple_vertex_normal_strategy.h>
#include <Eigen/Dense>

namespace cad_percept {
namespace collision_manifolds {
namespace offset_surface {

bool MultipleVertexNormalStrategy::execute(const cad_percept::cgal::Polyhedron& surface,
                                           double offset,
                                           cad_percept::cgal::Polyhedron* offset_surface) {
  surface_ = surface;
  getFaceNormals();

  // calculate all face normals with new method
  for (auto vertex = surface_.vertices_begin(); vertex != surface_.vertices_end(); ++vertex) {
    MultiNormal normal;
    calculateMultiNormal(vertex, &normal);
    vnormals_[vertex] = normal;
  }

  // output vertex statistics
  if (config_.statistics) {
    std::cout << "Faces per Vertex statistic:" << std::endl;
    for (auto statistic : vertex_faces_statistics_) {
      std::cout << "\t" << statistic.first << ": " << statistic.second << std::endl;
    }
  }

  // use simple vertex movement as a test.
  std::for_each(
      vnormals_.begin(), vnormals_.end(),
      std::bind(&MultipleVertexNormalStrategy::moveVertex, this, std::placeholders::_1, offset));

  *offset_surface = surface_;
  return true;
}

void MultipleVertexNormalStrategy::getFaceNormals() {
  boost::associative_property_map<FaceNormalMap> map_fnormals(fnormals_);
  CGAL::Polygon_mesh_processing::compute_face_normals(surface_, map_fnormals);
}

void MultipleVertexNormalStrategy::calculateMultiNormal(cgal::vertex_descriptor& vertex,
                                                        MultiNormal* multi_normal) {
  uint count_faces = 0;
  std::vector<Eigen::Vector3d> eigen_multi_normal;

  // iterate through all incident faces
  for (const cgal::halfedge_descriptor& d : CGAL::halfedges_around_source(vertex, surface_)) {
    Eigen::Vector3d face_normal;
    cgal::cgalVectorToEigenVector(fnormals_[d->face()], &face_normal);

    bool normal_used = false;

    // iterate through existing vectors and check if they are close enough.
    // if so, add current normal.
    for (auto& normal : eigen_multi_normal) {
      if (normal.cross(face_normal).norm() < config_.delta) {
        normal += face_normal;
        normal.normalize();
        normal_used = true;
        break;
      }
    }

    // if none of them was close enough, add another.
    if (!normal_used) {
      eigen_multi_normal.push_back(face_normal);
    }

    // keeping statistics.
    count_faces++;
  }

  std::cout << count_faces << " -> " << eigen_multi_normal.size() << std::endl;

  // Create statistics about vertices for info
  if (config_.statistics) {
    if (vertex_faces_statistics_.find(count_faces) == vertex_faces_statistics_.end()) {
      vertex_faces_statistics_[count_faces] = 0;
    }
    vertex_faces_statistics_[count_faces]++;
  }

  if (config_.use_twopass_normal_clustering) {
    // TODO(mpantic): Add twopass clustering.
  }

  // convert eigen normals back to cgal.
  multi_normal->resize(eigen_multi_normal.size());
  for (size_t i = 0; i < multi_normal->size(); ++i) {
    cgal::eigenVectorToCgalVector(eigen_multi_normal[i], &(multi_normal->at(i)));
  }
}

void MultipleVertexNormalStrategy::moveVertex(
    std::pair<cgal::vertex_descriptor, MultiNormal> vertex, const double offset) const {
  const cgal::Vector displacement = vertex.second[0] * offset;
  vertex.first->point() += displacement;  // In place change of vertex
}

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

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
    std::pair<cgal::vertex_descriptor, cgal::Vector> normal_pair;
    normal_pair.first = vertex;
    createNewVertex(vertex, &normal_pair.second);
    vnormals_.insert(normal_pair);
  }

  // output vertex statistics
  if (vertex_statistics_) {
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

bool MultipleVertexNormalStrategy::createNewVertex(cgal::vertex_descriptor& vertex,
                                                   cgal::Vector* normal) {
  uint count_faces = 0;
  std::vector<Eigen::Vector3d> multiple_normals;
  const double delta = 0.1;

  // iterate through all incident faces
  for (const cgal::halfedge_descriptor& d : CGAL::halfedges_around_source(vertex, surface_)) {
    Eigen::Vector3d face_normal;
    cgal::cgalVectorToEigenVector(fnormals_[d->face()], &face_normal);

    bool normal_used = false;
    // iterate through existing vectors and check if they are close enough.
    for (auto& multiple_normal : multiple_normals) {
      if (multiple_normal.cross(face_normal).norm() < delta) {
        multiple_normal += face_normal;
        multiple_normal.normalize();
        normal_used = true;
        break;
      }
    }

    if (!normal_used) {
      multiple_normals.push_back(face_normal);
    }

    count_faces++;
  }

  std::cout << count_faces << " -> " << multiple_normals.size() << std::endl;

  // Create statistics about vertices for info
  if (vertex_statistics_) {
    if (vertex_faces_statistics_.find(count_faces) == vertex_faces_statistics_.end()) {
      vertex_faces_statistics_[count_faces] = 0;
    }
    vertex_faces_statistics_[count_faces]++;
  }

  *normal = cgal::Vector(0.1, 0, 0);
  return true;
}

void MultipleVertexNormalStrategy::moveVertex(
    std::pair<cgal::vertex_descriptor, cgal::Vector> vertex, const double offset) const {
  const cgal::Vector displacement = vertex.second * offset;
  vertex.first->point() += displacement;  // In place change of vertex
}

}  // namespace offset_surface
}  // namespace collision_manifolds
}  // namespace cad_percept

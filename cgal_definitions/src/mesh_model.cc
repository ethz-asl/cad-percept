#include "cgal_definitions/mesh_model.h"

namespace cad_percept {
namespace cgal {

MeshModel::MeshModel(const std::string &off_path, bool verbose)
    : verbose_(verbose) {
  std::ifstream off_file(off_path.c_str(), std::ios::binary);
  if (!CGAL::read_off(off_file, P_)) {
    std::cerr << "Error: invalid STL file" << std::endl;
  }

  if (!P_.is_valid() || P_.empty()) {
    std::cerr << "Error: Invalid facegraph" << std::endl;
  }

  /**
 * Now we have loaded the geometric structure and need to conduct intersection
 * and distance queries. For this, we build an AABB tree.
 **/
  tree_ = std::make_shared<SurfaceMeshAABBTree>(CGAL::faces(P_).first,
                                                CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();
};

Intersection MeshModel::getIntersection(const Ray &query) const {
  if (verbose_) {
    std::cout << " %i intersections "
              << tree_->number_of_intersected_primitives(query) << std::endl;
  }

  // compute the closest intersection point and the distance
  SurfaceMeshRayIntersection intersection = tree_->first_intersection(query);
  if (intersection) {
    if (boost::get<Point>(&(intersection->first))) {
      Intersection intersection_result;
      Point p = *boost::get<Point>(&(intersection->first));
      intersection_result.point = p;
      intersection_result.surface_normal = getNormal(intersection->second);
      return intersection_result;
    }
  }
}

double MeshModel::getDistance(const Ray &query) const {
  // get the first intersection Point
  Point p = getIntersection(query).point;
  // get the distance from the ray origin
  Vector distance(query.source(), p);
  const double squared_distance = boost::get<double>(distance.squared_length());
  return sqrt(squared_distance);
}

PointAndPrimitiveId MeshModel::getClosestTriangle(Point &p) const {
  return tree_->closest_point_and_primitive(p);
}

PointAndPrimitiveId MeshModel::getClosestTriangle(double x, double y,
                                                  double z) const {
  Point pt = Point(x, y, z);
  return getClosestTriangle(pt);
}

Vector MeshModel::getNormal(SurfaceMesh::Face_handle &face_handle) const {
  Triangle intersected_triangle(
      face_handle->halfedge()->vertex()->point(),
      face_handle->halfedge()->next()->vertex()->point(),
      face_handle->halfedge()->next()->next()->vertex()->point());
  return intersected_triangle.supporting_plane().orthogonal_vector();
}

Vector MeshModel::getNormal(const PointAndPrimitiveId &ppid) const {
  return getNormal(ppid.second);
}

void MeshModel::transform(const Transformation &transform) {
  std::transform(P_.points_begin(), P_.points_end(), P_.points_begin(),
                 transform);
  tree_ = std::make_shared<SurfaceMeshAABBTree>(CGAL::faces(P_).first,
                                                CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();
}

int MeshModel::size() const { return P_.size_of_facets(); }
}
}

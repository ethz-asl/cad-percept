#include "cgal_definitions/mesh_model.h"

namespace cad_percept {
namespace mesh_model {

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
  tree_ =
      std::make_shared<Tree>(CGAL::faces(P_).first, CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();
};

Intersection MeshModel::getIntersection(
    const kindr::minimal::QuatTransformationTemplate<double> &pose) const {
  // Building the ray to query for intersections.
  // The ray will be defined by a rayOrigin and a point on the ray.
  // We put the point on the ray to 1, 0, 0 and apply the transformation.
  Eigen::Matrix<double, 3, Eigen::Dynamic> ray_points =
      Eigen::MatrixXd::Zero(3, 2);
  ray_points(0, 1) = 1;
  ray_points = pose.transformVectorized(ray_points);
  Point ray_origin =
      Point(ray_points(0, 0), ray_points(1, 0), ray_points(2, 0));
  Point onRay = Point(ray_points(0, 1), ray_points(1, 1), ray_points(2, 1));
  Ray query(ray_origin, onRay);

  if (verbose_)
    ROS_INFO(" %i intersections ",
             tree_->number_of_intersected_primitives(query));

  // compute the closest intersection point and the distance
  Ray_intersection intersection = tree_->first_intersection(query);
  if (intersection) {
    if (boost::get<Point>(&(intersection->first))) {
      Intersection intersection_result;
      Point p = *boost::get<Point>(&(intersection->first));
      intersection_result.point = Eigen::Vector3d(p.x(), p.y(), p.z());
      // Polyhedron::Face_handle face_handle =
      //   *boost::get<Polyhedron::Face_handle>(&(intersection->second));
      // Polyhedron::Face_handle face_handle =
      //    tree_->first_intersected_primitive(query);
      Polyhedron::Face_handle face_handle = intersection->second;
      Triangle intersected_triangle(
          face_handle->halfedge()->vertex()->point(),
          face_handle->halfedge()->next()->vertex()->point(),
          face_handle->halfedge()->next()->next()->vertex()->point());
      Vector normal =
          intersected_triangle.supporting_plane().orthogonal_vector();
      intersection_result.surface_normal =
          Eigen::Vector3d(normal.x(), normal.y(), normal.z());
      return intersection_result;
    }
  }
}

double MeshModel::getDistance(
    const kindr::minimal::QuatTransformationTemplate<double> &pose) const {
  // get the first intersection Point
  Eigen::Vector3d p = getIntersection(pose).point;
  Eigen::Vector3d o = pose.getPosition();
  Point ray_origin = Point(o(0), o(1), o(2));
  Point intersection = Point(p(0), p(1), p(2));
  // get the squared distance
  Vector distance(ray_origin, intersection);
  const double squared_distance = boost::get<double>(distance.squared_length());
  return sqrt(squared_distance);
}

Point_and_primitive_id MeshModel::getClosestTriangle(
    double x, double y, double z) const {
  Point pt = Point(x, y, z);
  return tree_->closest_point_and_primitive(pt);
}

Eigen::Vector3d MeshModel::getNormal(const Point_and_primitive_id &ppid) const {
  Polyhedron::Face_handle face_handle = ppid.second;
  Triangle intersected_triangle(
      face_handle->halfedge()->vertex()->point(),
      face_handle->halfedge()->next()->vertex()->point(),
      face_handle->halfedge()->next()->next()->vertex()->point());
  Vector normal = intersected_triangle.supporting_plane().orthogonal_vector();
  return Eigen::Vector3d(normal.x(), normal.y(), normal.z());
}

void MeshModel::transform(const Transformation &transform) {
  std::transform(P_.points_begin(),
                 P_.points_end(),
                 P_.points_begin(),
                 transform);
  tree_ =
      std::make_shared<Tree>(CGAL::faces(P_).first, CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();
}

int MeshModel::size() const {
  return P_.size_of_facets();
}

}
}

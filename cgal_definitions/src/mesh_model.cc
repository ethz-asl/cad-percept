#include "cgal_definitions/mesh_model.h"
#include <cgal_definitions/cgal_meshing_typedefs.h>
namespace cad_percept {
namespace cgal {

MeshModel::MeshModel(Polyhedron &p) : MeshModel(p, false) {}

MeshModel::MeshModel(Polyhedron &p, bool verbose) : P_(std::move(p)), verbose_(verbose) {
  // Initlaize trees and facet index
  tree_ = std::make_shared<PolyhedronAABBTree>(CGAL::faces(P_).first, CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();

  initializeFacetIndices();  // set fixed facet IDs for whole class
}

// Static factory method.
bool MeshModel::create(const std::string &off_path, MeshModel::Ptr *ptr, bool verbose) {
  Polyhedron p;
  std::ifstream off_file(off_path.c_str(), std::ios::binary);

  // Check if file is accessible
  if (!off_file.good()) {
    std::cerr << "Error: File not readable " << off_path << std::endl;
    return false;
  }

  // check if cgal could read it
  if (!CGAL::read_off(off_file, p)) {
    std::cerr << "Error: invalid STL file" << std::endl;
    return false;
  }

  // check polyhedron structure
  if (!p.is_valid() || p.empty()) {
    std::cerr << "Error: Invalid facegraph" << std::endl;
    return false;
  }

  // Create new object and assign (note cannot use make_shared here because constructor is private)
  ptr->reset(new MeshModel(p, verbose));
  return true;
}

// checks if there is an intersection at all
bool MeshModel::isIntersection(const Ray &query) const {
  PolyhedronRayIntersection intersection = tree_->first_intersection(query);
  if (intersection) {
    return true;
  } else {
    return false;
  }
}

// this will throw an exception if there is no intersection, so check first by
// using function isIntersection()
Intersection MeshModel::getIntersection(const Ray &query) const {
  if (verbose_) {
    std::cout << " %i intersections " << tree_->number_of_intersected_primitives(query)
              << std::endl;
  }

  // compute the closest intersection point and the distance
  PolyhedronRayIntersection intersection = tree_->first_intersection(query);
  Intersection intersection_result;
  try {
    Point p = *boost::get<Point>(&(intersection->first));
    intersection_result.intersected_point = p;
    intersection_result.surface_normal = getNormal(intersection->second);
  } catch (...) {
    std::cout << "There is no intersection result. Use isIntersection() first." << std::endl;
  }

  return intersection_result;
}

// check first if there is intersection with isIntersection()
double MeshModel::getDistance(const Ray &query) const {
  // get the first intersection Point
  Point p = getIntersection(query).intersected_point;
  // get the distance from the ray origin
  Vector distance(query.source(), p);  // creates a vector "distance"
  const double squared_distance = boost::get<double>(distance.squared_length());
  return sqrt(squared_distance);
}

PointAndPrimitiveId MeshModel::getClosestTriangle(const Point &p) const {
  return tree_->closest_point_and_primitive(p);  // primitive Id is Facet_handle
}

PointAndPrimitiveId MeshModel::getClosestTriangle(const double x, const double y,
                                                  const double z) const {
  Point pt = Point(x, y, z);
  return getClosestTriangle(pt);
}

// directed to positive side of h
Vector MeshModel::getNormal(const Polyhedron::Face_handle &face_handle) const {
  // introduce the triangle with 3 points:
  Triangle intersected_triangle(face_handle->halfedge()->vertex()->point(),
                                face_handle->halfedge()->next()->vertex()->point(),
                                face_handle->halfedge()->next()->next()->vertex()->point());
  return intersected_triangle.supporting_plane().orthogonal_vector();
}

Vector MeshModel::getNormal(const PointAndPrimitiveId &ppid) const {
  return getNormal(ppid.second);
}

void MeshModel::transform(const Transformation &transform) {
  std::transform(P_.points_begin(), P_.points_end(), P_.points_begin(), transform);
  // create updated AABBTree:
  tree_ = std::make_shared<PolyhedronAABBTree>(CGAL::faces(P_).first, CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();
}

int MeshModel::size() const { return P_.size_of_facets(); }

Polyhedron MeshModel::getMesh() const { return P_; }

void MeshModel::initializeFacetIndices() {
  // for vertices there exist CGAL::set_halfedgeds_items_id(m), but not for
  // facets
  std::size_t i = 0;
  for (Polyhedron::Facet_iterator facet = P_.facets_begin(); facet != P_.facets_end(); ++facet) {
    facet->id() = i++;
  }
}

int MeshModel::getFacetIndex(Polyhedron::Facet_handle &handle) {
  int facet_id;
  facet_id = handle->id();
  return facet_id;
}
}  // namespace cgal
}  // namespace cad_percept

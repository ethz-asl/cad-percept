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
  tree_ = std::make_shared<PolyhedronAABBTree>(CGAL::faces(P_).first,
                                               CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();

  initializeFacetIndices(); // set fixed facet IDs for whole class
};

MeshModel::MeshModel(const Polyhedron &mesh, bool verbose) : verbose_(verbose) {
  P_ = mesh;
  tree_ = std::make_shared<PolyhedronAABBTree>(CGAL::faces(P_).first,
                                               CGAL::faces(P_).second,
                                               P_);
  tree_->accelerate_distance_queries();
  initializeFacetIndices();
}

// checks if there is an intersection at all
bool MeshModel::isIntersection(const Ray &query) const {
  PolyhedronRayIntersection intersection = tree_->first_intersection(query);
  if (intersection) {
    return true;
  }
  else {
    return false;
  }
}

// this will throw an exception if there is no intersection, so check first by using function isIntersection()
Intersection MeshModel::getIntersection(const Ray &query) const {
  if (verbose_) {
    std::cout << " %i intersections "
              << tree_->number_of_intersected_primitives(query) << std::endl;
  }

  // compute the closest intersection point and the distance
  PolyhedronRayIntersection intersection = tree_->first_intersection(query);
  Intersection intersection_result;
  try {
    Point p = *boost::get<Point>(&(intersection->first));
    intersection_result.intersected_point = p;
    intersection_result.surface_normal = getNormal(intersection->second);
  }
  catch(...) {
    std::cout << "There is no intersection result. Use isIntersection() first." << std::endl;
  }

  return intersection_result;
}

// check first if there is intersection with isIntersection()
double MeshModel::getDistance(const Ray &query) const {
  // get the first intersection Point
  Point p = getIntersection(query).intersected_point;
  // get the distance from the ray origin
  Vector distance(query.source(), p); // creates a vector "distance"
  const double squared_distance = boost::get<double>(distance.squared_length());
  return sqrt(squared_distance);
}

PointAndPrimitiveId MeshModel::getClosestTriangle(const Point &p) const {
  return tree_->closest_point_and_primitive(p); // primitive Id is Facet_handle
}

PointAndPrimitiveId MeshModel::getClosestTriangle(const double x,
                                                  const double y,
                                                  const double z) const {
  Point pt = Point(x, y, z);
  return getClosestTriangle(pt);
}

// directed to positive side of h
Vector MeshModel::getNormal(const Polyhedron::Face_handle &face_handle) const { //Polyhedron::Face_handle does not really exist?!
  // introduce the triangle with 3 points, works with any 3 points of Polyhedron:
  Triangle intersected_triangle(
      face_handle->halfedge()->vertex()->point(),
      face_handle->halfedge()->next()->vertex()->point(),
      face_handle->halfedge()->next()->next()->vertex()->point());
  Vector n = intersected_triangle.supporting_plane().orthogonal_vector();
  n = n / sqrt(n.squared_length());
  return n;
}

Vector MeshModel::getNormal(const PointAndPrimitiveId &ppid) const {
  return getNormal(ppid.second);
}

void MeshModel::transform(const Transformation &transform) {
  std::transform(P_.points_begin(), P_.points_end(), P_.points_begin(),
                 transform);
  // create updated AABBTree:
  tree_ = std::make_shared<PolyhedronAABBTree>(CGAL::faces(P_).first,
                                               CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();
}

int MeshModel::size() const { return P_.size_of_facets(); }

Polyhedron MeshModel::getMesh() const { return P_; }

void MeshModel::initializeFacetIndices() {
  // for vertices there exist CGAL::set_halfedgeds_items_id(m), but not for facets
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

std::map<int, Vector> MeshModel::computeNormals() {
  // computes all Polyhedron normals and return only normal map with ID
  std::map<int, Vector> normals;
  std::map<face_descriptor, Vector> fnormals;
  std::map<vertex_descriptor, Vector> vnormals;

  CGAL::Polygon_mesh_processing::compute_normals(P_,
                                                boost::make_assoc_property_map(vnormals),
                                                boost::make_assoc_property_map(fnormals));
  std::cout << "Face normals :" << std::endl;
  for(face_descriptor fd: faces(P_)){ // faces returns iterator range over faces, range over all face indices
    std::cout << fnormals[fd] << std::endl;
    std::cout << fd->id() << std::endl; // not sure why this even works
    normals.insert(std::pair<int, Vector>(fd->id(), fnormals[fd]));
  }
  std::cout << "Vertex normals :" << std::endl;
  for(vertex_descriptor vd: vertices(P_)){
    std::cout << vnormals[vd] << std::endl;
  }

  /* Just for testing - remove later */
  Polyhedron::Facet_handle fh = P_.facets_begin();
  fh++;
  fh++;
  fh++;
  Vector test_normal;  
  test_normal = getNormal(fh);
  std::cout << "First normal with getNormal is: " << test_normal << std::endl;
  test_normal = computeFaceNormal(fh);
  std::cout << "First normal with descriptor is: " << test_normal << std::endl;
  test_normal = computeFaceNormal2(fh);
  std::cout << "First normal with cross product is: " << test_normal << std::endl;

  // Polyhedron can save plane equation with operator:
  // https://doc.cgal.org/latest/Polyhedron/Polyhedron_2polyhedron_prog_normals_8cpp-example.html

  /***************************/

  return normals;
}

Vector MeshModel::computeFaceNormal(face_descriptor fd) {
  // unclear how to get face descriptor from facet
  Vector face_normal;
  face_normal = CGAL::Polygon_mesh_processing::compute_face_normal(fd, P_); // not sure if this works properly with handle
  return face_normal;
}

Vector MeshModel::computeFaceNormal2(const Polyhedron::Facet_handle &facet_handle) {
  Polyhedron::Halfedge_handle he = facet_handle->halfedge();
  Point p0 = he->vertex()->point();
  Point p1 = he->next()->vertex()->point();
  Point p2 = he->next()->next()->vertex()->point();
  // alternatively: Vector n = CGAL::normal(p0, p1, p2); check directions
  Vector n = CGAL::cross_product(p0-p2, p1-p2);
  n = n / sqrt(n.squared_length());
  return n;
}

//void MeshModel::mergeCoplanarFacets() {
  
//}

}
}


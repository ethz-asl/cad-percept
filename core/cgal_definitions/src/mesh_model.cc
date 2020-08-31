#include "cgal_definitions/mesh_model.h"

namespace cad_percept {
namespace cgal {

// A modifier creating a triangle with the incremental builder.
template <class HDS>
void MeshFromJSON<HDS>::operator()(HDS &hds) {
  CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
  B.begin_surface(j_["vertex"].size(), j_["face"].size());
  // add all vertices first
  for (auto &[id, pos] : j_["vertex"].items()) {
    B.add_vertex(Point(pos["x"], pos["y"], pos["z"]));
    vertex_to_index_[id] = vertex_order_.size();
    vertex_order_.push_back(id);
  }
  for (auto &[id, vertices] : j_["face"].items()) {
    B.begin_facet();
    for (std::string vertex : vertices) {
      B.add_vertex_to_facet(vertex_to_index_[vertex]);
    }
    B.end_facet();
    triangle_order_.push_back(id);
  }
  B.end_surface();
}

template <class HDS>
void MeshFromJSON<HDS>::setJson(const nlohmann::json &j) {
  j_ = j;
}

template <class HDS>
std::vector<std::string> MeshFromJSON<HDS>::getVertexIds() {
  return vertex_order_;
}

template <class HDS>
std::vector<std::string> MeshFromJSON<HDS>::getTriangleIds() {
  return triangle_order_;
}

MeshModel::MeshModel(Polyhedron &p, bool verbose) : P_(std::move(p)), verbose_(verbose) {
  // Initlaize trees and facet index
  tree_ = std::make_shared<PolyhedronAABBTree>(CGAL::faces(P_).first, CGAL::faces(P_).second, P_);
  tree_->accelerate_distance_queries();

  initializeFacetIndices();  // set fixed facet IDs for whole class
}

// Static factory methods.
bool MeshModel::create(Polyhedron &p, MeshModel::Ptr *ptr, bool verbose) {
  // check polyhedron structure
  if (!p.is_valid() || p.empty()) {
    std::cerr << "Error: Invalid facegraph" << std::endl;
    return false;
  }
  // Create new object and assign (note cannot use make_shared here because constructor is private)
  ptr->reset(new MeshModel(p, verbose));
  return true;
}

bool MeshModel::create(nlohmann::json &j, MeshModel::Ptr *ptr, bool verbose) {
  // read mesh
  Polyhedron p;
  MeshFromJSON<HalfedgeDS> mesh_generator;
  mesh_generator.setJson(j);
  p.delegate(mesh_generator);

  // Create new object and assign (note cannot use make_shared here because constructor is
  // private)
  ptr->reset(new MeshModel(p, verbose));

  // read IDs
  (*ptr)->setTriangleIds(mesh_generator.getTriangleIds());
  return true;
}

bool MeshModel::create(const std::string &filepath, MeshModel::Ptr *ptr, bool verbose) {
  if (0 == filepath.compare(filepath.length() - 4, 4, ".off")) {
    // .off files
    std::ifstream off_file(filepath.c_str(), std::ios::binary);

    // Check if file is accessible
    if (!off_file.good()) {
      std::cerr << "Error: File not readable " << filepath << std::endl;
      return false;
    }
    Polyhedron p;
    // check if cgal could read it
    if (!CGAL::read_off(off_file, p)) {
      std::cerr << "Error: invalid STL file" << std::endl;
      return false;
    }

    return create(p, ptr, verbose);
  } else if (0 == filepath.compare(filepath.length() - 5, 5, ".json")) {
    std::ifstream json_file(filepath.c_str());
    nlohmann::json j;
    json_file >> j;

    return create(j, ptr, verbose);
  } else {
    std::cerr << "File " << filepath << " does not match any of the supported filetypes "
              << "(.off, .json)." << std::endl;
  }
  return false;
}

void MeshModel::initializeFacetIndices() {
  // for vertices there exist CGAL::set_halfedgeds_items_id(m), but not for facets
  int i = 0;
  for (Polyhedron::Facet_iterator facet = P_.facets_begin(); facet != P_.facets_end(); ++facet) {
    facet->id() = i;
    facetToHandle_[std::to_string(i)] = &(*facet);
    facetIdxToId_[i] = std::to_string(i);
    ++i;
  }
}

void MeshModel::setTriangleIds(const std::vector<std::string> &triangle_ids) {
  if (triangle_ids.size() != P_.size_of_facets()) {
    std::cerr << "[MeshModel] Cannot set triangle ids. Mesh has " << P_.size_of_facets()
              << " facets and " << triangle_ids.size() << "ids were provided." << std::endl;
    return;
  }
  std::unordered_map<int, std::string> facetIdxToId_new;
  std::unordered_map<std::string, Polyhedron::Facet_handle> facetToHandle_new;
  std::unordered_map<std::string, std::string> facetToPlane_new;
  std::unordered_multimap<std::string, std::string> planeToFacets_new;
  for (uint facet_idx = 0; facet_idx < P_.size_of_facets(); ++facet_idx) {
    std::string new_id = triangle_ids[facet_idx];
    std::string old_id = facetIdxToId_[facet_idx];
    facetIdxToId_new[facet_idx] = new_id;
    facetToHandle_new[new_id] = facetToHandle_[old_id];
    if (facetToPlane_.size() != 0) {
      // planes are already defined
      facetToPlane_new[new_id] = facetToPlane_[old_id];
      auto range = planeToFacets_.equal_range(old_id);
      for (auto it = range.first; it != range.second; ++it) {
        planeToFacets_new.insert(std::make_pair(new_id, it->second));
      }
    }
  }
  facetIdxToId_ = facetIdxToId_new;
  facetToHandle_ = facetToHandle_new;
  facetToPlane_ = facetToPlane_new;
  planeToFacets_ = planeToFacets_new;
}

bool MeshModel::isCorrectId(const std::string &facet_id) const {
  return facetToHandle_.find(facet_id) != facetToHandle_.end();
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
  Vector n = intersected_triangle.supporting_plane().orthogonal_vector();
  // normalize vector
  n = n / sqrt(n.squared_length());
  return n;
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
Polyhedron &MeshModel::getMeshRef() { return P_; }
PolyhedronPtr MeshModel::getMeshPtr() { return PolyhedronPtr(&P_); }

std::string MeshModel::getIdFromFacetHandle(const Polyhedron::Facet_handle &handle) const {
  return facetIdxToId_.at(handle->id());
}

Polyhedron::Facet_handle MeshModel::getFacetHandleFromId(const std::string facet_id) const {
  return facetToHandle_.at(facet_id);
}

std::map<std::string, Vector> MeshModel::computeNormals() const {
  std::map<std::string, Vector> normals;
  std::map<face_descriptor, Vector> fnormals;
  std::map<vertex_descriptor, Vector> vnormals;

  CGAL::Polygon_mesh_processing::compute_normals(P_, boost::make_assoc_property_map(vnormals),
                                                 boost::make_assoc_property_map(fnormals));
  // faces returns iterator range over faces, range over all face indices
  for (face_descriptor fd : faces(P_)) {
    normals[facetIdxToId_.at(fd->id())] = fnormals[fd];
  }
  return normals;
}

Vector MeshModel::computeFaceNormal(face_descriptor &fd) const {
  // unclear how to get face descriptor from facet, but works with Facet_handle
  Vector face_normal;
  face_normal = CGAL::Polygon_mesh_processing::compute_face_normal(
      fd, P_);  // not sure if this works properly with handle
  return face_normal;
}

Vector MeshModel::computeFaceNormal2(const Polyhedron::Facet_handle &facet_handle) const {
  Polyhedron::Halfedge_handle he = facet_handle->halfedge();
  Point p0 = he->vertex()->point();
  Point p1 = he->next()->vertex()->point();
  Point p2 = he->next()->next()->vertex()->point();
  // alternatively: Vector n = CGAL::normal(p0, p1, p2); check directions
  Vector n = CGAL::cross_product(p0 - p2, p1 - p2);
  n = n / sqrt(n.squared_length());
  return n;
}

bool MeshModel::coplanar(const Polyhedron::Halfedge_handle &h1,
                         const Polyhedron::Halfedge_handle &h2, double eps) const {
  // not sure about rounding errors when computing normals, so for eps = 0 use:
  if (eps == 0) {
    // check coplanarity of at least three points from the new triangle
    if (CGAL::coplanar(h1->vertex()->point(), h1->next()->vertex()->point(),
                       h1->next()->next()->vertex()->point(), h2->vertex()->point()) &&
        CGAL::coplanar(h1->vertex()->point(), h1->next()->vertex()->point(),
                       h1->next()->next()->vertex()->point(), h2->next()->vertex()->point()) &&
        CGAL::coplanar(h1->vertex()->point(), h1->next()->vertex()->point(),
                       h1->next()->next()->vertex()->point(),
                       h2->next()->next()->vertex()->point()))
      return true;
    else
      return false;
  } else {
    Vector n1 = computeFaceNormal2(h1->facet());
    Vector n2 = computeFaceNormal2(h2->facet());
    // calculate angle between the two normal vectors
    double phi = acos(CGAL::to_double(CGAL::scalar_product(n1, n2)));  // rad
    if (phi > eps)
      return false;
    else
      return true;
  }
}

void MeshModel::findCoplanarFacets(std::string facet_id, std::unordered_set<std::string> *result,
                                   const double eps) {
  // we can use the "result" pointer here directly even if there are already results there,
  // because they are coplanar and we check results, there is no need to check again
  if (facetToHandle_.find(facet_id) == facetToHandle_.end()) {
    std::cerr << "Facet ID not part of this MeshModel." << std::endl;
    return;
  }
  // queue of coplanar facets of which we need to check the neighbors
  std::queue<Polyhedron::Facet_handle> coplanar_to_check;
  Polyhedron::Facet_handle start_handle = getFacetHandleFromId(facet_id);
  coplanar_to_check.push(start_handle);
  result->insert(facet_id);

  while (!coplanar_to_check.empty()) {
    Polyhedron::Facet_handle handle = coplanar_to_check.front();
    coplanar_to_check.pop();
    Polyhedron::Halfedge_around_facet_circulator hit = handle->facet_begin();
    do {
      if (!(hit->is_border_edge())) {
        // check if neighboring facets are coplanar
        if (coplanar(hit, hit->opposite(), eps)) {  // play with eps to get whole plane
          if (CGAL::circulator_size(hit->opposite()->vertex_begin()) >= 3 &&
              CGAL::circulator_size(hit->vertex_begin()) >= 3 &&
              hit->facet()->id() != hit->opposite()->facet()->id()) {
            std::string next_facet_id = facetIdxToId_[hit->opposite()->facet()->id()];
            if (result->find(next_facet_id) == result->end()) {
              result->insert(next_facet_id);
              coplanar_to_check.push(hit->opposite()->facet());
            }
          }
        }
      }
    } while (++hit != handle->facet_begin());
  }
}

void MeshModel::findAllCoplanarFacets(
    std::unordered_map<std::string, std::string> *facetToPlane,
    std::unordered_multimap<std::string, std::string> *planeToFacets, const double eps) {
  // only do actual computation once
  if (facetToPlane_.size() == 0) {
    int plane_counter = 0;  // we generate ids from counting up an integer
    for (uint current_facet_idx = 0; current_facet_idx < P_.size_of_facets(); ++current_facet_idx) {
      // only find coplanar facets if facet is not already associated to a plane
      if (facetToPlane_.count(facetIdxToId_[current_facet_idx]) == 0) {
        // we found a facet that is part of a new plane
        std::string plane_id = std::to_string(plane_counter);
        std::unordered_set<std::string> current_coplanar_facets;
        findCoplanarFacets(facetIdxToId_[current_facet_idx], &current_coplanar_facets, eps);
        // add result into lookup maps
        for (auto facet_id : current_coplanar_facets) {
          facetToPlane_[facet_id] = plane_id;
          planeToFacets_.insert(std::make_pair(plane_id, facet_id));
        }
        ++plane_counter;
      }
    }
    std::cout << plane_counter << " planes from coplanar facets found." << std::endl;
  }
  // Deep copy both maps
  for (auto elem : facetToPlane_) {
    facetToPlane->insert(std::make_pair(elem.first, elem.second));
  }
  for (auto elem : planeToFacets_) {
    planeToFacets->insert(std::make_pair(elem.first, elem.second));
  }
}

Plane MeshModel::getPlane(const Polyhedron::Facet_handle &f) const {
  return Plane(f->halfedge()->vertex()->point(), f->halfedge()->next()->vertex()->point(),
               f->halfedge()->next()->next()->vertex()->point());
}

Plane MeshModel::getPlane(const std::string facet_id) const {
  return getPlane(getFacetHandleFromId(facet_id));
}

Triangle MeshModel::getTriangle(const Polyhedron::Facet_handle &f) const {
  return Triangle(f->halfedge()->vertex()->point(), f->halfedge()->next()->vertex()->point(),
                  f->halfedge()->next()->next()->vertex()->point());
}

Triangle MeshModel::getTriangle(const std::string facet_id) const {
  return getTriangle(getFacetHandleFromId(facet_id));
}

double MeshModel::getArea() const {
  FT area;
  area = CGAL::Polygon_mesh_processing::area(P_);
  return CGAL::to_double(area);
}

double MeshModel::getArea(const Polyhedron::Facet_handle &f) const {
  FT area;
  area = CGAL::Polygon_mesh_processing::face_area(f, P_);
  return CGAL::to_double(area);
}

double MeshModel::getArea(const std::string facet_id) const {
  return getArea(getFacetHandleFromId(facet_id));
}

double MeshModel::squaredDistance(const Point &point) const {
  FT sqd = tree_->squared_distance(point);
  return CGAL::to_double(sqd);
}

}  // namespace cgal
}  // namespace cad_percept

#ifndef CGAL_DEFINITIONS_MESH_MODEL_H
#define CGAL_DEFINITIONS_MESH_MODEL_H

#include <math.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "cgal_typedefs.h"

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <nlohmann/json.hpp>

namespace cad_percept {
namespace cgal {

struct Intersection {
  Point intersected_point;
  Vector surface_normal;
};

// For reading from json files
template <class HDS>
class MeshFromJSON : public CGAL::Modifier_base<HDS> {
 public:
  MeshFromJSON() {}

  void operator()(HDS &hds);
  void setJson(const nlohmann::json &j);
  std::vector<std::string> getVertexIds();
  std::vector<std::string> getTriangleIds();

 private:
  nlohmann::json j_;
  // keep track of which vertex was inserted when
  std::vector<std::string> vertex_order_;
  std::unordered_map<std::string, int> vertex_to_index_;
  // keep track of which triangle was inserted when
  std::vector<std::string> triangle_order_;
};

class MeshModel {
 public:
  typedef std::shared_ptr<MeshModel> Ptr;
  static bool create(const std::string &filepath, MeshModel::Ptr *meshmodel_ptr,
                     bool verbose = false);
  static bool create(Polyhedron &p, MeshModel::Ptr *meshmodel_ptr, bool verbose = false);
  static bool create(nlohmann::json &j, MeshModel::Ptr *meshmodel_ptr, bool verbose = false);

  /**
   * Check if there is an intersection.
   */
  bool isIntersection(const Ray &query) const;

  /**
   * Get the intersection between the ray and the mesh model. Function will throw an exception
   * if there is no intersection, so check first by using function isIntersection().
   */
  Intersection getIntersection(const Ray &query) const;

  /**
   * Get the distance of the intersection with the mesh from the ray origin. Check first if
   * there is intersection with isIntersection().
   */
  double getDistance(const Ray &query) const;

  /**
   * Get closest point on surface and surface id to a given point.
   */
  PointAndPrimitiveId getClosestTriangle(const Point &p) const;
  PointAndPrimitiveId getClosestTriangle(const double x, const double y, const double z) const;

  /**
   * Get normal of primitive.
   */
  Vector getNormal(const Polyhedron::Face_handle &face_handle) const;
  Vector getNormal(const PointAndPrimitiveId &ppid) const;
  Vector computeFaceNormal(face_descriptor &fd) const;
  Vector computeFaceNormal2(const Polyhedron::Facet_handle &facet_handle) const;

  /**
   * Computes all Polyhedron normals and returns normal map with ID.
   */
  std::map<std::string, Vector> computeNormals() const;

  /**
   * Transform the mesh model.
   */
  void transform(const Transformation &transform);

  /**
   * Return size of mesh (number of facet primitives).
   */
  int size() const;

  /**
   * Return mesh
   */
  Polyhedron getMesh() const;
  Polyhedron* getMeshNonConst(){
    return &P_;
  }

  /**
   * Check coplanarity of two facets described by halfedge handle h1 and h2
   */
  bool coplanar(const Polyhedron::Halfedge_handle &h1, const Polyhedron::Halfedge_handle &h2,
                double eps) const;

  /**
   * IDs and Lookup handling
   */
  void setTriangleIds(const std::vector<std::string> &triangle_ids);

  bool isCorrectId(const std::string &facet_id) const;

  /**
   * Facet Handle <-> Facet ID
   */
  Polyhedron::Facet_handle getFacetHandleFromId(const std::string facet_id) const;
  std::string getIdFromFacetHandle(const Polyhedron::Facet_handle &handle) const;

  /**
   * Get a triangle type from a facet handle or ID
   */
  Triangle getTriangle(const Polyhedron::Facet_handle &f) const;
  Triangle getTriangle(const std::string facet_id) const;

  /**
   * Get a plane type from a facet handle or ID
   */
  Plane getPlane(const Polyhedron::Facet_handle &f) const;
  Plane getPlane(const std::string facet_id) const;

  /**
   * Find all coplanar facet IDs to a given facet ID. A tolerance eps is used to measure
   * coplanarity because of numerical errors.
   */
  void findCoplanarFacets(std::string facet_id, std::unordered_set<std::string> *result,
                          const double eps);

  /**
   * Find all coplanar facets in the model and save associations to two maps.
   * This function is super slow if mesh consists of many primitives. Only execute it once in
   * beginning.
   */
  void findAllCoplanarFacets(std::unordered_map<std::string, std::string> *facetToPlane,
                             std::unordered_multimap<std::string, std::string> *planeToFacets,
                             const double eps);

  /**
   * Get the surface area of the complete Polyhedron.
   */
  double getArea() const;

  /**
   * Get the surface area of a specific facet.
   */
  double getArea(const Polyhedron::Facet_handle &f) const;
  double getArea(const std::string facet_id) const;

  /**
   * Compute squared distance from point to closest mesh facet.
   */
  double squaredDistance(const Point &point) const;

 private:
  MeshModel(Polyhedron &p, bool verbose);  // Constructor to be used by factory method
  Polyhedron P_;
  std::shared_ptr<PolyhedronAABBTree> tree_;
  bool verbose_;

  /**
   * Iterate through all facets and associate an ID.
   */
  void initializeFacetIndices();

  // ID associations between elements
  std::unordered_map<int, std::string> facetIdxToId_;
  std::unordered_map<std::string, Polyhedron::Facet_handle> facetToHandle_;
  std::unordered_map<std::string, std::string> facetToPlane_;
  std::unordered_multimap<std::string, std::string> planeToFacets_;
};
}  // namespace cgal
}  // namespace cad_percept
#endif

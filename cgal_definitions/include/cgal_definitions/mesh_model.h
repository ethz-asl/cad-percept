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

namespace cad_percept {
namespace cgal {

struct Intersection {
  Point intersected_point;
  Vector surface_normal;
};

class MeshModel {
 public:
  typedef std::shared_ptr<MeshModel> Ptr;
  static bool create(const std::string &off_pathm, MeshModel::Ptr *meshmodel_ptr,
                     bool verbose = false);
  static bool create(Polyhedron &p, MeshModel::Ptr *meshmodel_ptr, bool verbose = false);

  /**
   * Check if there is an intersection
   */
  bool isIntersection(const Ray &query) const;

  /**
   * Get the intersection between the ray and the mesh model.
   */
  Intersection getIntersection(const Ray &query) const;
  /**
   * Get the distance of the intersection with the mesh from the ray origin.
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
  std::map<int, Vector> computeNormals() const;
  Vector computeFaceNormal(face_descriptor &fd) const;
  Vector computeFaceNormal2(const Polyhedron::Facet_handle &facet_handle) const;

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

  /**
   * ID Lookups
   */
  Polyhedron::Facet_handle getFacetHandleFromId(const uint facet_id);
  int getIdFromFacetHandle(const Polyhedron::Facet_handle &handle);

  /**
   * Check coplanarity of two facets described by halfedge handle h1 and h2
   */
  bool coplanar(const Polyhedron::Halfedge_handle &h1, const Polyhedron::Halfedge_handle &h2,
                double eps) const;

  void printFacetsOfHalfedges();

  /**
   * Compute Plane from facet_handle
   */
  Plane getPlaneFromHandle(Polyhedron::Facet_handle &f) const;
  Plane getPlaneFromID(uint facet_id);

  /**
   *  Compute Triangle from facet
   */
  Triangle getTriangleFromHandle(Polyhedron::Facet_handle &f) const;
  Triangle getTriangleFromID(uint facet_id);

  void findCoplanarFacets(uint facet_id, std::unordered_set<int> *result, const double eps);

  /**
   * This function is super slow. Only execute it once in beginning.
   */
  void findAllCoplanarFacets(std::unordered_map<int, int> *facetToPlane,
                             std::unordered_multimap<int, int> *planeToFacets, const double eps);

  double getArea() const;

  double getArea(Polyhedron::Facet_handle &f) const;
  double getArea(uint facet_id);

  /**
   * Compute squared distance from point to closest mesh facet
   */
  double squaredDistance(const Point &point) const;

 private:
  MeshModel(Polyhedron &p, bool verbose);  // Constructor to be used by factory method
  Polyhedron P_;
  std::shared_ptr<PolyhedronAABBTree> tree_;
  bool verbose_;

  void initializeFacetIndices();

  // ID associations between elements
  std::unordered_map<int, int> facetToPlane_;
  std::unordered_map<int, Polyhedron::Facet_handle> facetIdToHandle_;
  std::unordered_multimap<int, int> planeToFacets_;
};
}  // namespace cgal
}  // namespace cad_percept
#endif

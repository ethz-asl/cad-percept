#ifndef CGAL_DEFINITIONS_MESH_MODEL_H
#define CGAL_DEFINITIONS_MESH_MODEL_H

#include <math.h>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <map>

#include "cgal_typedefs.h"

namespace cad_percept {
namespace cgal {

struct Intersection {
  Point intersected_point;
  Vector surface_normal;
};

class MeshModel {
 public:
  MeshModel(const std::string &off_pathm, bool verbose = false);

  MeshModel(const Polyhedron &mesh, bool verbose = false);
  
  MeshModel(); // necessary to create class object which will be initialized later

  void init(const std::string &off_path, bool verbose = false);

  void init(const Polyhedron &mesh, bool verbose = false);

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
 * Get closest point on surface and surface id to a given point. (renamed since it works on every Polyhedron, not only triangle)
 */
  PointAndPrimitiveId getClosestPrimitive(const Point &p) const;
  PointAndPrimitiveId getClosestPrimitive(const double x, const double y,
                                         const double z) const;

  /**
   * Get normal (different implementations, same result)
   */
  Vector getNormal(const Polyhedron::Facet_handle &facet_handle) const;
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
   * Get facet iterator
   */
  Polyhedron::Facet_iterator getFacetIterator();

  int getFacetIndex(const Polyhedron::Facet_handle &handle);

  /**
   * Check coplanarity of two facets described by halfedge handle h1 and h2
   */
  bool coplanar(const Polyhedron::Halfedge_handle &h1, const Polyhedron::Halfedge_handle &h2, double eps) const;

  void printFacetsOfHalfedges();

  /**
   * Compute Plane from facet_handle
   */
  Plane getPlane(Polyhedron::Facet_handle &f) const;

  /**
   * Merge coplanar facets of MeshModel variable P_ and return new Polyhedron P_out and ID
   * associations. MeshModel class is kept as it is.
   */
  void mergeCoplanarFacets(Polyhedron *P_out, std::multimap<int, int> *merge_associations) const;

  double getArea() const;

  /**
   * Compute squared distance from point to closest mesh facet
   */
  double squaredDistance(const Point &point) const;


 private:
  Polyhedron P_;
  std::shared_ptr<PolyhedronAABBTree> tree_;
  bool verbose_;

  void initializeFacetIndices();

};
}
}
#endif

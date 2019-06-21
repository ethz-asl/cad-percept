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

// Static Deviations:
typedef boost::graph_traits<Polyhedron>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<Polyhedron>::face_descriptor   face_descriptor;

struct Intersection {
  Point intersected_point;
  Vector surface_normal;
};

class MeshModel {
 public:
  MeshModel(const std::string &off_pathm, bool verbose = false);

  MeshModel(const Polyhedron &mesh, bool verbose = false);
  
  MeshModel();

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
 * Get closest point on surface and surface id to a given point.
 */
  PointAndPrimitiveId getClosestPrimitive(const Point &p) const;
  PointAndPrimitiveId getClosestPrimitive(const double x, const double y,
                                         const double z) const;

  /**
 * Get normal of primitive.
 */
  Vector getNormal(const Polyhedron::Face_handle &face_handle) const;
  Vector getNormal(const PointAndPrimitiveId &ppid) const;

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

  int getFacetIndex(Polyhedron::Facet_handle &handle);

  std::map<int, Vector> computeNormals();

  Vector computeFaceNormal(face_descriptor fd);

  Vector computeFaceNormal2(const Polyhedron::Facet_handle &facet_handle);

  bool coplanar(const Polyhedron::Halfedge_handle &h1, const Polyhedron::Halfedge_handle &h2, double eps);

  void printFacetsOfHalfedges();

  Plane getPlane(Polyhedron::Facet_handle &f) const;

  void mergeCoplanarFacets(Polyhedron *P_out, std::multimap<int, int> *merge_associations);


 private:
  Polyhedron P_;
  std::shared_ptr<PolyhedronAABBTree> tree_;
  bool verbose_;

  void initializeFacetIndices();
};
}
}
#endif

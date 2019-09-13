#ifndef CGAL_DEFINITIONS_MESH_MODEL_H
#define CGAL_DEFINITIONS_MESH_MODEL_H

#include <math.h>
#include <fstream>
#include <iostream>

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

  MeshModel(Polyhedron &p);
  MeshModel(Polyhedron &p, bool verbose);  // Constructor to be used by factory method

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

  PolyhedronPtr getMeshPtr();

  /**
   * Get facet iterator
   */
  Polyhedron::Facet_iterator getFacetIterator();

  void initializeFacetIndices();

  int getFacetIndex(Polyhedron::Facet_handle &handle);

 private:
  MeshModel() {}  // private default constructor
  Polyhedron P_;
  std::shared_ptr<PolyhedronAABBTree> tree_;
  bool verbose_;
};
}  // namespace cgal
}  // namespace cad_percept
#endif

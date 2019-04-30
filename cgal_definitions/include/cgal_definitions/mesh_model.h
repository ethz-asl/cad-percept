#ifndef CGAL_DEFINITIONS_MESH_MODEL_H
#define CGAL_DEFINITIONS_MESH_MODEL_H

#include <math.h>
#include <fstream>
#include <iostream>

#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Geometry>

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

  /**
 * Get the Intersection point with the arhcitecture model from the pose of the
 * pointlaser. The pointlaser is assumed to point into the x-axis direction in
 * it's own frame.
 */
  Intersection getIntersection(
      const kindr::minimal::QuatTransformationTemplate<double> &pose) const;
  /**
 * Get the epxected measured distance according to the architecture model from
 * the given  pose of the pointlaser. The pointlaser is assumed to point into
 * the x-axis direction in it's own frame.
 */
  double getDistance(
      const kindr::minimal::QuatTransformationTemplate<double> &pose) const;

  /**
 * Get closest point on surface and surface id to a given point.
 */
  PointAndPrimitiveId getClosestTriangle(double x, double y, double z) const;

  /**
 * Get normal of primitive.
 */
  Eigen::Vector3d getNormal(const PointAndPrimitiveId &ppid) const;

  /**
 * Transform the architect model.
 */
  void transform(const Transformation &transform);

  /**
 * Return size of architect model (number of primitives).
 */
  int size() const;

 private:
  SurfaceMesh P_;
  std::shared_ptr<SurfaceMeshAABBTree> tree_;
  bool verbose_;
};
}
}
#endif

#ifndef CPT_MESHING_MESHER_INTERFACE_H
#define CPT_MESHING_MESHER_INTERFACE_H

#include <cgal_definitions/cgal_typedefs.h>
#include <cpt_meshing/pcl_typedefs.h>
#include <iostream>

namespace cad_percept {
namespace meshing {

// Data structure for performance measurements.
typedef struct {
  float processing_time;  // milli-seconds
  uint num_vertices;      // Number of resulting vertices
  uint num_points;        // Number of input points (cleaned pointcloud)
  // todo:: add more.
} MeshPerformanceCounters;

std::ostream& operator<<(std::ostream& os, const MeshPerformanceCounters& s) {
  return (os << "t = " << s.processing_time << "\t n_points = " << s.num_points
             << "\t n_vertices = " << s.num_vertices);
}

class MesherInterface {
 public:
  /*
   * Adds a point cloud and its normals to the meshing buffer.
   * Depending on the mesher, this might overwrite older data.
   * This call should return fast, so not much processing happening here.
   */
  virtual void addPointCloud(const InputPointCloud::Ptr& input,
                             const InputNormals::Ptr& normals) = 0;

  /*
   *  Returns true and a valid mesh if meshing was successful,
   *  otherwise returns false.
   *  Note: This call might block for a long time (processing happens here)
   */
  virtual bool getMesh(cad_percept::cgal::Polyhedron* output,
                       MeshPerformanceCounters* counters = nullptr) = 0;
};

}  // namespace meshing
}  // namespace cad_percept
#endif  // CPT_MESHING_MESHER_INTERFACE_H

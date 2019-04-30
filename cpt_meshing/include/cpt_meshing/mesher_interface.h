#ifndef CPT_MESHING_MESHER_INTERFACE_H
#define CPT_MESHING_MESHER_INTERFACE_H

#include <cpt_meshing/pcl_typedefs.h>
#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace meshing {

// Data structure for performance measurements.
typedef struct {
  float processing_time;  //milli-seconds
  // todo:: add more.
} MeshPerformanceCounters;

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

} // namespace meshing
} // namespace cad_percept
#endif //CPT_MESHING_MESHER_INTERFACE_H

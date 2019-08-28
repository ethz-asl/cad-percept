/*
 * Abstract Baseclass for a mesher that operates
 * on the full point cloud (no incremental advancing front meshing etc).
 *
 */
#ifndef CPT_MESHING_ABSTRACT_SIMPLE_MESHER_H
#define CPT_MESHING_ABSTRACT_SIMPLE_MESHER_H

#include <cpt_meshing/mesher_interface.h>
namespace cad_percept {
namespace meshing {

class AbstractSimpleMesher : public MesherInterface {
 public:
  bool addPointCloud(const InputPointCloud::Ptr& input, const InputNormals::Ptr& normals) {
    points_ = input;
    normals_ = normals;
    return inputPointsValid();
  }

  virtual bool getMesh(cad_percept::cgal::Polyhedron* output,
                       MeshPerformanceCounters* counters) = 0;

 protected:
  bool inputPointsValid() {
    // check if data is assigned
    if (points_ != nullptr && normals_ != nullptr) {
      // check if length is the same
      return points_->points.size() == normals_->points.size();
    }

    // any other case, return false
    return false;
  }

  InputPointCloud::Ptr points_;
  InputNormals::Ptr normals_;
};
}
}
#endif  // CPT_MESHING_ABSTRACT_SIMPLE_MESHER_H

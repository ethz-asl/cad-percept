#ifndef CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_
#define CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_

#include <ros/ros.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>

namespace cad_percept {
namespace object_detection {

class DetectionMatcher {
 public:
  DetectionMatcher(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  ~DetectionMatcher() = default;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  cgal::MeshModel mesh_model_;
};

}
}

#endif  // CAD_PERCEPT_CPT_OBJECT_DETECTION_INCLUDE_CPT_OBJECT_DETECTION_DETECTION_MATCHER_H_

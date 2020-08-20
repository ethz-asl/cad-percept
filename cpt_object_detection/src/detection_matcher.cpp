#include "cpt_object_detection/detection_matcher.h"

namespace cad_percept {
namespace object_detection {

DetectionMatcher::DetectionMatcher(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      mesh_model_(nh_private.param<std::string>("off_model","fail")) {
  LOG(INFO) << "[DetectionMatcher] Object mesh with "
            << mesh_model_.getMesh().size_of_facets() << " facets and "
            << mesh_model_.getMesh().size_of_vertices()
            << " vertices";
}

}
}
#include "cpt_pointlaser_common/utils.h"

#include <minkindr_conversions/kindr_tf.h>
#include <tf/transform_listener.h>

namespace cad_percept {
namespace pointlaser_common {

kindr::minimal::QuatTransformation getTF(std::string from, std::string to) {
  tf::StampedTransform transform;
  kindr::minimal::QuatTransformation ret;
  tf::TransformListener transform_listener_;
  transform_listener_.lookupTransform(from, to, ros::Time(0), transform);
  tf::transformTFToKindr(transform, &ret);
  return ret;
}

}  // namespace pointlaser_common
}  // namespace cad_percept
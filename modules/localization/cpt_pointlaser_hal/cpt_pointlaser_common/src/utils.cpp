#include "cpt_pointlaser_common/utils.h"

#include <minkindr_conversions/kindr_tf.h>

namespace cad_percept {
namespace pointlaser_common {

kindr::minimal::QuatTransformation getTF(const tf::TransformListener &transform_listener,
                                         std::string from, std::string to) {
  tf::StampedTransform transform;
  kindr::minimal::QuatTransformation ret;
  transform_listener.lookupTransform(from, to, ros::Time(0), transform);
  tf::transformTFToKindr(transform, &ret);
  return ret;
}

}  // namespace pointlaser_common
}  // namespace cad_percept
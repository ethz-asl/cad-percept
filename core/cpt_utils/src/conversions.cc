#include "cpt_utils/conversions.h"

namespace cad_percept {
namespace cpt_utils {

/**
 * Transform between Eigen and ros.
 */
void toRosTransform(const Eigen::Vector3d &translation, const Eigen::Quaterniond &quaternion,
                    geometry_msgs::Transform *msg_out) {
  tf::vectorEigenToMsg(translation, msg_out->translation);
  tf::quaternionEigenToMsg(quaternion, msg_out->rotation);
}

}  // namespace cpt_utils
}  // namespace cad_percept

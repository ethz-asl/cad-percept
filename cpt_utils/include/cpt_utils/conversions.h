#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <eigen_conversions/eigen_msg.h>
#include <glog/logging.h>
#include <geometry_msgs/Transform.h>

#include <cgal_definitions/cgal_typedefs.h>

namespace cad_percept {
namespace cpt_utils {

// TODO(gawela): This could be merged with cgal_conversions into cpt_conversions.
/**
 * Transform between Eigen and ros.
 */
void toRosTransform(const Eigen::Vector3d &translation, const Eigen::Quaterniond &rotation,
                      geometry_msgs::Transform *msg_out);

}  // namespace cpt_utils
}  // namespace cad_percept

#endif  // CONVERSIONS_H_
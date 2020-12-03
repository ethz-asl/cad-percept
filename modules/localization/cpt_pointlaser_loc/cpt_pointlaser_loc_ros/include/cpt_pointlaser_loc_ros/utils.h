#ifndef CPT_POINTLASER_LOC_ROS_UTILS_H_
#define CPT_POINTLASER_LOC_ROS_UTILS_H_

#include <kindr/minimal/quat-transformation.h>

namespace cad_percept {
namespace pointlaser_loc_ros {

///
/// \brief Returns the most recent TF in the minkindr QuatTransformation format.
/// 
/// \param from TF source.
/// \param to   TF target.
/// \return Most recent TF in the minkindr QuatTransformation format.
///
kindr::minimal::QuatTransformation getTF(std::string from, std::string to);

}  // namespace pointlaser_loc_ros
}  // namespace cad_percept
#endif

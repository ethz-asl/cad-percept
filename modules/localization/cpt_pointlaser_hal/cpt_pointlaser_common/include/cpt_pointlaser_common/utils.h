#ifndef CPT_POINTLASER_COMMON_UTILS_H_
#define CPT_POINTLASER_COMMON_UTILS_H_

#include <kindr/minimal/quat-transformation.h>
#include <tf/transform_listener.h>

namespace cad_percept {
namespace pointlaser_common {

///
/// \brief Returns the most recent TF in the minkindr QuatTransformation format.
///
/// \param transform_listener Transform listener that should be used to look up the transform.
/// \param from               TF source.
/// \param to                 TF target.
/// \return Most recent TF in the minkindr QuatTransformation format.
///
kindr::minimal::QuatTransformation getTF(const tf::TransformListener &transform_listener,
                                         std::string from, std::string to);

}  // namespace pointlaser_common
}  // namespace cad_percept
#endif

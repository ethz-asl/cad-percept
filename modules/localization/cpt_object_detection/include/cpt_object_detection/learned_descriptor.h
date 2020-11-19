#ifndef CPT_OBJECT_DETECTION_LEARNED_DESCRIPTOR_H_
#define CPT_OBJECT_DETECTION_LEARNED_DESCRIPTOR_H_

#include <modelify/pcl_common.h>
#include <modelify/registration_toolbox/fast_global_registration.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

struct LearnedDescriptor {
  float learned_descriptor[32];
  static size_t descriptorSize() { return 32; };
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(LearnedDescriptor,
                                  (float, learned_descriptor, learned_descriptor));

namespace modelify {

template <>
inline size_t getMatchingDescriptorLength<LearnedDescriptor>() {
  return LearnedDescriptor::descriptorSize();
};

template <>
inline float registration_toolbox::fast_global_registration::getDescriptorBin<LearnedDescriptor>(
    const LearnedDescriptor& descriptor, const size_t bin) {
  return descriptor.learned_descriptor[bin];
};

}  // namespace modelify

#endif  // CPT_OBJECT_DETECTION_LEARNED_DESCRIPTOR_H_

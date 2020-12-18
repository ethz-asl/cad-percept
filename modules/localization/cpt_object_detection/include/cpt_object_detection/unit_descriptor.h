#ifndef CPT_OBJECT_DETECTION_UNIFORM_DESCRIPTOR_H_
#define CPT_OBJECT_DETECTION_UNIFORM_DESCRIPTOR_H_

#include <modelify/pcl_common.h>
#include <pcl/pcl_macros.h>
//#include <pcl/point_types.h>

struct UnitDescriptor {
  float unit_descriptor[1] = {0.0f};
  static size_t descriptorSize() { return 1; };
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(UnitDescriptor,
                                  (float, unit_descriptor, unit_descriptor));

namespace modelify {

template <>
inline size_t getMatchingDescriptorLength<UnitDescriptor>() {
  return UnitDescriptor::descriptorSize();
};

template <>
inline float registration_toolbox::fast_global_registration::getDescriptorBin<UnitDescriptor>(
    const UnitDescriptor& descriptor, const size_t bin) {
  return descriptor.unit_descriptor[bin];
};

}  // namespace modelify

#endif  // CPT_OBJECT_DETECTION_UNIFORM_DESCRIPTOR_H_

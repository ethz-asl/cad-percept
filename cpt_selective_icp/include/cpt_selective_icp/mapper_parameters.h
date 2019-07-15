// A struct reading all the mapper parameters from parameter server as simplification

#include <pointmatcher_ros/get_params_from_server.h>

#ifndef CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_
#define CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_

namespace cad_percept {
namespace cpt_selective_icp {

struct MapperParameters {

  MapperParameters();
  ~MapperParameters();

  // Parameters

  int min_reading_point_coun(getParam<int>("minReadingPointCount", 2000));
};

}
}
#endif // CPT_SELECTIVE_ICP_MAPPER_PARAMETERS_H_
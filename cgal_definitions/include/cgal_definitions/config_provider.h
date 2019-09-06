#ifndef CGAL_DEFINITIONS_CONFIG_PROVIDER_H_
#define CGAL_DEFINITIONS_CONFIG_PROVIDER_H_
#include <iostream>
#include "not_implemented_exception.h"

namespace cad_percept {

/*
 * Base interface for a configuration provider.
 * Used to break dependency on a specific framework (e.g. ROS).
 */
template <class ParamId>
class ConfigProvider {
 public:
  /*
   * Gets a parameter based on it's id.
   * Returns default value if param couldn't be found.
   */
  template <class T>
  T getParam(ParamId name, T& default_value) {
    // This function cannot be pure virtual as it is templated.
  }

  /*
   * Returns true if param exists, false otherwise.
   */
  virtual bool hasParam(ParamId name) = 0;
};

}  // namespace cad_percept

#endif  // CGAL_DEFINITIONS_CONFIG_PROVIDER_H_

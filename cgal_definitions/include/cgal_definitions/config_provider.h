#ifndef CGAL_DEFINITIONS_CONFIG_PROVIDER_H_
#define CGAL_DEFINITIONS_CONFIG_PROVIDER_H_
#include <cgal_definitions/not_implemented_exception.h>
#include <iostream>
#include <memory>
#include <vector>

#define VIRTUAL_GETTER_FOR_TYPE(t)                             \
  virtual t getParam(std::string, const t& default_value) = 0; \
  virtual std::vector<t> getParam(std::string, const std::vector<t>& default_value) = 0;

#define GETTER_FOR_TYPE(t)                                                                  \
  t getParam(std::string name, const t& default_value) override {                           \
    return getParamImpl(name, default_value);                                               \
  };                                                                                        \
  std::vector<t> getParam(std::string name, const std::vector<t>& default_value) override { \
    return getParamImpl(name, default_value);                                               \
  };

namespace cad_percept {

/*
 * Base interface for a configuration provider.
 * Used to break dependency on a specific framework (e.g. ROS).
 */
class ConfigProvider {
 public:
  typedef std::shared_ptr<ConfigProvider> Ptr;

  /*
   * The poor man's polymorphic templating:
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   *
   * Problem: C++ does not support pure virtual functions for templates
   * So we have to hack something to make it clear upfront what types are
   * supported, so that we can keep them pure virtual.
   *
   * Therefore we use macros to generate the appropriate functions and virtual functions.
   * The macros assume that the following templated private method exists in the subclass:
   *
   *  template <class T>
   *  T getParamImpl(std::string name, const T& default_value);
   *
   *  See cpt_ros for an example.
   */
  VIRTUAL_GETTER_FOR_TYPE(int)
  VIRTUAL_GETTER_FOR_TYPE(double)
  VIRTUAL_GETTER_FOR_TYPE(bool)
  VIRTUAL_GETTER_FOR_TYPE(std::string)

  /*
   * Returns true if param exists, false otherwise.
   */
  virtual bool hasParam(std::string name) = 0;
};

}  // namespace cad_percept

#endif  // CGAL_DEFINITIONS_CONFIG_PROVIDER_H_

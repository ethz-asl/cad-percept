#ifndef CPT_GEOMESH_TOOLS_FILTER_CENTER_NODE_H
#define CPT_GEOMESH_TOOLS_FILTER_CENTER_NODE_H
#include <string>
#include <utility>
#include <vector>

namespace cad_percept::geomesh_tools {

class FilterCenterNode {
 public:
  void filter(std::string path, std::pair<double, double> roi_start,
              std::pair<double, double> roi_end);
};

}  // namespace cad_percept::geomesh_tools

#endif  // CPT_GEOMESH_TOOLS_FILTER_CENTER_NODE_H

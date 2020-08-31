#ifndef CPT_GEOMESH_TOOLS_FILTER_CENTER_NODE_H
#define CPT_GEOMESH_TOOLS_FILTER_CENTER_NODE_H
#include <geotf/geodetic_converter.h>

#include <Eigen/Dense>
#include <string>
#include <utility>
#include <vector>

namespace cad_percept::geomesh_tools {

class FilterCenterNode {
 public:
  void init() {
    geotf_.initFromRosParam();
    if (!geotf_.hasFrame(name_3857_)) {
      geotf_.addFrameByEPSG(name_3857_, 3857);
    }
    //geotf_.addFrameByGCSCode("GPS", "WGS84");
    //geotf_.addFrameByENUOrigin("ENU", 46.58120,8.38509,2219.5);
  }

  void setMap(const std::string& path, const std::string& frame);
  void setROI(std::vector<Eigen::Vector3d> roi, const std::string& frame);

  void clipMesh();
  void generateMesh(const std::string& output_path, const std::string& output_frame);

 private:
  std::string name_3857_ = "Pseudo-Mercator";
  std::vector<Eigen::Vector3d> roi_points_3857_;  // Default frame EPSG:3857
  cad_percept::cgal::MeshModel::Ptr model_;

  geotf::GeodeticConverter geotf_;
};

}  // namespace cad_percept::geomesh_tools

#endif  // CPT_GEOMESH_TOOLS_FILTER_CENTER_NODE_H

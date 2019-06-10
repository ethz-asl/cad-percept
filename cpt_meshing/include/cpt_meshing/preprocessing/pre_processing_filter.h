#ifndef CPT_MESHING_PRE_PROCESSING_FILTER_H
#define CPT_MESHING_PRE_PROCESSING_FILTER_H
#include <cpt_meshing/pcl_typedefs.h>

namespace cad_percept {
namespace meshing {

class PreProcessingFilter {
 public:
  PreProcessingFilter();

  void run(const InputPointCloud::ConstPtr input,
           InputPointCloud::Ptr output,
           InputNormals::Ptr normals);

  // Adds a simple box filter to constrain axes
  void addBoxFilter(const std::string& axis,
                    double min,
                    double max);

  // adds a voxel integration filter to the pipeline
  void addVoxelFilter(double x_leaf, double y_leaf, double z_leaf);

  void setNormalEstimationRadius(const double radius){
    normal_search_radius_ = radius;
  }


 private:
  // Convenience method for normal estimation.
  void estimateNormals(InputPointCloud::ConstPtr input,
                       InputNormals::Ptr normals);

  std::vector<std::shared_ptr<InputPointCloudFilter>> filter_list_;
  double normal_search_radius_; // radius for estimating normals in [m]
};
}
}

#endif //CPT_MESHING_PRE_PROCESSING_FILTER_H

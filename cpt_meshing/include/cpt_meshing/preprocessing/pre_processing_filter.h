#ifndef CPT_MESHING_PRE_PROCESSING_FILTER_H
#define CPT_MESHING_PRE_PROCESSING_FILTER_H
#include <cpt_meshing/pcl_typedefs.h>

namespace cad_percept {
namespace meshing {

class PreProcessingFilter {
 public:
  void run(const InputPointCloud::ConstPtr input,
           InputPointCloud::Ptr output,
           InputNormals::Ptr normals);

  void addBoxFilter(const std::string& axis,
                    double min,
                    double max);

  void addVoxelFilter(double x_leaf, double y_leaf, double z_leaf);

 private:
  void estimateNormals(InputPointCloud::ConstPtr input, InputNormals::Ptr normals);
  std::vector<std::shared_ptr<InputPointCloudFilter>> filter_list_;
};
}
}

#endif //CPT_MESHING_PRE_PROCESSING_FILTER_H

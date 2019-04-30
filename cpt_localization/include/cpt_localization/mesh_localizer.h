#ifndef CPT_LOCALIZATION_MESH_LOCALIZER_H
#define CPT_LOCALIZATION_MESH_LOCALIZER_H

#include <pcl_ros/point_cloud.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>

namespace cad_percept {
namespace localization {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Associations {
  Eigen::Matrix3Xd points_from;
  Eigen::Matrix3Xd points_to;
  Eigen::VectorXd distances;
};

class MeshLocalizer {
 public:
  MeshLocalizer();
  MeshLocalizer(const cad_percept::cgal::SurfaceMesh &mesh);

  ~MeshLocalizer();

  /*
   Associates point clouds with mesh.
   */
  Associations associatePointCloud(const PointCloud &pc_msg) const;

  /*
  Iteratively minimize error and re-associatepoint cloud with mesh. ICM =
  iterative closest mesh.
   */
  void icm(const PointCloud &pc_msg);

 private:
  std::shared_ptr<cgal::MeshModel> mesh_model_;
};

}
}
#endif
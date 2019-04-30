#ifndef CPT_LOCALIZATION_MESH_LOCALIZER_H
#define CPT_LOCALIZATION_MESH_LOCALIZER_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/Expression.h>

#include <kindr/minimal/quat-transformation.h>

#include <pcl_ros/point_cloud.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>

namespace cad_percept {
namespace localization {

typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

static double multiplyVectorsImplementation(Eigen::Vector3d a,
                                            Eigen::Vector3d b,
                                            gtsam::OptionalJacobian<1,3> Ha,
                                            gtsam::OptionalJacobian<1,3> Hb) {

  if(Ha)
    *Ha = b.transpose();

  if(Hb)
    *Hb = a.transpose();

  return a.transpose() * b;
}

static gtsam::Expression<double> multiplyVectors(const gtsam::Expression<Eigen::Vector3d>& C1,
                                                 const gtsam::Expression<Eigen::Vector3d>& C2) {
  return gtsam::Expression<double>(&multiplyVectorsImplementation, C1, C2);
}

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
  void icm(const PointCloud &pc_msg, const SE3 &initial_pose);

 private:
  std::shared_ptr<cgal::MeshModel> mesh_model_;
  gtsam::NonlinearFactorGraph factor_graph_;
};

}
}
#endif
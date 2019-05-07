#ifndef CPT_UTILS_H
#define CPT_UTILS_H

#include <kindr/minimal/quat-transformation-gtsam.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_conversions/mesh_conversions.h>

namespace cad_percept {
namespace cpt_utils {

typedef kindr::minimal::QuatTransformationTemplate<double> SE3;
typedef SE3::Rotation SO3;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct Associations {
  Eigen::Matrix3Xd points_from;
  Eigen::Matrix3Xd points_to;
  Eigen::VectorXd distances;
};

// Associate point-cloud with architect model.
Associations associatePointCloud(const PointCloud &pc_msg, const cgal::MeshModel &mesh_model_);

// Function to transform architect model.
void transformModel(const Eigen::Matrix4d &transformation, cgal::MeshModel &mesh_model_);

// Return model size.
int size(const cgal::MeshModel &mesh_model_);

// Return architect model as point cloud.
PointCloud getModelAsPointCloud(const cgal::MeshModel &mesh_model_);

Eigen::Vector3d vectorToEigenVector(const cgal::Vector &v);

cgal::Vector eigenVectorToVector(const Eigen::Vector3d &ve);

}
}



#endif // CPT_UTILS_H
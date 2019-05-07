#include "cpt_utils/cpt_utils.h"

namespace cad_percept {
namespace cpt_utils {

Associations associatePointCloud(const PointCloud &pc_msg, const cgal::MeshModel &mesh_model_) {
  // Convert pointcloud msg
  std::cout << "Associating pointcloud of size " << pc_msg.width << " "
            << pc_msg.height << std::endl;
  Associations associations;
  associations.points_from.resize(3, pc_msg.width);
  associations.points_to.resize(3, pc_msg.width);
  associations.distances.resize(pc_msg.width);
  for (size_t i = 0u; i < pc_msg.width; ++i) {

    cgal::PointAndPrimitiveId ppid =
        mesh_model_.getClosestTriangle(pc_msg[i].x,
                                        pc_msg[i].y,
                                        pc_msg[i].z);
    cgal::Point pt = ppid.first;
    associations.points_from(0, i) = pc_msg[i].x;
    associations.points_from(1, i) = pc_msg[i].y;
    associations.points_from(2, i) = pc_msg[i].z;

    // Raycast into direction of triangle normal.
    Eigen::Vector3d normal = vectorToEigenVector(mesh_model_.getNormal(ppid)); 
    normal.normalize();
    Eigen::Vector3d relative = Eigen::Vector3d(pt.x(), pt.y(), pt.z())
        - Eigen::Vector3d(pc_msg[i].x, pc_msg[i].y, pc_msg[i].z);
    Eigen::Vector3d direction = normal.dot(relative) * normal;

    associations.points_to(0, i) =
        associations.points_from(0, i) + direction(0);
    associations.points_to(1, i) =
        associations.points_from(1, i) + direction(1);
    associations.points_to(2, i) =
        associations.points_from(2, i) + direction(2);
    associations.distances(i) = direction.norm();
  }
  return associations;
}

void transformModel(const Eigen::Matrix4d &transformation, cgal::MeshModel &mesh_model_) {
  cgal::Transformation
      trans(transformation(0, 0),
            transformation(0, 1),
            transformation(0, 2),
            transformation(0, 3),
            transformation(1, 0),
            transformation(1, 1),
            transformation(1, 2),
            transformation(1, 3),
            transformation(2, 0),
            transformation(2, 1),
            transformation(2, 2),
            transformation(2, 3),
            1.0);
  mesh_model_.transform(trans);
}

int size(const cgal::MeshModel &mesh_model_) {
  return mesh_model_.size();
}

PointCloud getModelAsPointCloud(const cgal::MeshModel &mesh_model_) {
  PointCloud pc_msg;
  cgal::Polyhedron m;
  m = mesh_model_.getMesh();
  cgal::meshToVerticePointCloud(m, &pc_msg);

  return pc_msg;
}

/* Use the conversion functions in cgal_conversions package later! */

Eigen::Vector3d vectorToEigenVector(const cgal::Vector &v) {
  return Eigen::Vector3d(v.x(), v.y(), v.z());
}

cgal::Vector eigenVectorToVector(const Eigen::Vector3d &ve) {
  return cgal::Vector(ve(0,0), ve(1,0), ve(2,0));
}

/* ---- */

}
}
#include "cpt_localization/mesh_localizer.h"

namespace cad_percept {
namespace localization {

Associations MeshLocalizer::associatePointCloud(const PointCloud &pc_msg) const {
  std::cout << "Associating pointcloud of size " << pc_msg.width << " "
            << pc_msg.height << std::endl;
  Associations associations;
  associations.points_from.resize(3, pc_msg.width);
  associations.points_to.resize(3, pc_msg.width);
  associations.distances.resize(pc_msg.width);
  for (size_t i = 0u; i < pc_msg.width; ++i) {

    cgal::PointAndPrimitiveId ppid =
        mesh_model_->getClosestTriangle(pc_msg[i].x,
                                        pc_msg[i].y,
                                        pc_msg[i].z);
    cgal::Point pt = ppid.first;
    associations.points_from(0, i) = pc_msg[i].x;
    associations.points_from(1, i) = pc_msg[i].y;
    associations.points_from(2, i) = pc_msg[i].z;

    // Raycast into direction of triangle normal.
    Eigen::Vector3d normal = mesh_model_->getNormal(ppid);
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

void MeshLocalizer::icm(const PointCloud &pc_msg) {

}

}
}
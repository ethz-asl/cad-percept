#include "cpt_utils/cpt_utils.h"

namespace cad_percept {
namespace cpt_utils {

Associations associatePointCloud(const PointCloud &pc_msg, cgal::MeshModel *mesh_model) {
  // Convert point cloud msg
  std::cout << "Associating pointcloud of size " << pc_msg.width << " x "
            << pc_msg.height << std::endl;
  Associations associations;
  associations.points_from.resize(3, pc_msg.width); //3 rows, width columns
  associations.points_to.resize(3, pc_msg.width);
  associations.distances.resize(pc_msg.width);
  associations.triangles_to.resize(pc_msg.width);
  for (size_t i = 0u; i < pc_msg.width; ++i) {
    // loop through all points of point cloud

    cgal::PointAndPrimitiveId ppid =
        mesh_model->getClosestTriangle(pc_msg[i].x,
                                        pc_msg[i].y,
                                        pc_msg[i].z);
    cgal::Point pt = ppid.first;

    int triangle_id = mesh_model->getFacetIndex(ppid.second);

    associations.points_from(0, i) = pc_msg[i].x;
    associations.points_from(1, i) = pc_msg[i].y;
    associations.points_from(2, i) = pc_msg[i].z;

    // Raycast into direction of triangle normal.
    Eigen::Vector3d normal = cgal::cgalVectorToEigenVector(mesh_model->getNormal(ppid)); 
    normal.normalize();
    Eigen::Vector3d relative = Eigen::Vector3d(pt.x(), pt.y(), pt.z())
        - Eigen::Vector3d(pc_msg[i].x, pc_msg[i].y, pc_msg[i].z);
    Eigen::Vector3d direction = normal.dot(relative) * normal; // but relative is already in direction of normal because of getClosestTriangle (?!)

    associations.points_to(0, i) =
        associations.points_from(0, i) + direction(0); // points_to should be pt, right?
    associations.points_to(1, i) =
        associations.points_from(1, i) + direction(1);
    associations.points_to(2, i) =
        associations.points_from(2, i) + direction(2);
    associations.distances(i) = direction.norm();
    associations.triangles_to(i) = triangle_id;
  }
  return associations;
}

}
}

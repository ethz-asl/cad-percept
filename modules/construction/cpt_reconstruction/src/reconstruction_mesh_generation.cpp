#include "cpt_reconstruction/reconstruction_mesh_generation.h"

namespace cad_percept {
namespace cpt_reconstruction {
MeshGeneration::MeshGeneration(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle) {
  subscriber_ = nodeHandle_.subscribe("classified_shapes", 1000,
                                      &MeshGeneration::messageCallback, this);
  ros::spin();
}

void MeshGeneration::messageCallback(
    const ::cpt_reconstruction::classified_shapes &msg) {
  for (int i = 0; i < msg.id.size(); i++) {
    ROS_INFO("Received shape: %d", msg.classes.at(i));
  }
}

void combineMeshes(const pcl::PolygonMesh &mesh, pcl::PolygonMesh &mesh_all) {
  // pcl::PolygonMesh::concatenate(mesh_all, mesh); ???
  // Source:
  // https://github.com/PointCloudLibrary/pcl/blob/master/common/include/pcl/PolygonMesh.h
  mesh_all.header.stamp = std::max(mesh_all.header.stamp, mesh.header.stamp);

  const auto point_offset = mesh_all.cloud.width * mesh_all.cloud.height;

  // Transform them to PointXYZ and back to PCLPointCloud (to remove redundant
  // fields)
  pcl::PointCloud<pcl::PointXYZ> mesh_all_cloud;
  pcl::PointCloud<pcl::PointXYZ> mesh_cloud;
  pcl::fromPCLPointCloud2(mesh_all.cloud, mesh_all_cloud);
  pcl::fromPCLPointCloud2(mesh.cloud, mesh_cloud);
  pcl::PCLPointCloud2 mesh_all_cloud2;
  pcl::PCLPointCloud2 mesh_cloud2;
  pcl::toPCLPointCloud2(mesh_cloud, mesh_cloud2);
  pcl::toPCLPointCloud2(mesh_all_cloud, mesh_all_cloud2);

  // Conatenate point clouds
  pcl::PCLPointCloud2 new_cloud;
  pcl::concatenatePointCloud(mesh_all_cloud2, mesh_cloud2, new_cloud);
  mesh_all.cloud = new_cloud;

  // Concatenate polygons
  std::transform(
      mesh.polygons.begin(), mesh.polygons.end(),
      std::back_inserter(mesh_all.polygons), [point_offset](auto polygon) {
        std::transform(polygon.vertices.begin(), polygon.vertices.end(),
                       polygon.vertices.begin(),
                       [point_offset](auto &point_idx) {
                         return point_idx + point_offset;
                       });
        return polygon;
      });
}

}  // namespace cpt_reconstruction
}  // namespace cad_percept
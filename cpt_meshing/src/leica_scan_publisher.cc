#include <CGAL/IO/Polyhedron_iostream.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <cpt_meshing/leica_scan_publisher.h>
#include <cpt_meshing/mesher/delaunay_2d_mesher.h>
#include <cpt_meshing/mesher/delaunay_3d_mesher.h>
#include <cpt_meshing/preprocessing/pre_processing_filter.h>
#include <fstream>
#include <string>
#include <vector>

namespace cad_percept {
namespace meshing {

LeicaScanPublisher::LeicaScanPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Settings only needed here.
  bool publish_on_start_ = nh_private_.param<bool>("publish_on_start", true);
  bool latch_topic_ = nh_private_.param<bool>("latch_topic", true);

  // Set-up of node.
  pub_mesh_ = nh_private_.advertise<cgal_msgs::TriangleMeshStamped>("mesh_out", 1, latch_topic_);
  // publish_service_ =
  //  nh_private_.advertiseService("publish", &MeshPublisher::triggerPublishMesh, this);
  pub_pcl_ = nh_private_.advertise<InputPointCloud>("points_out", 1, latch_topic_);
  default_filename_ = nh_private_.param<std::string>("default_filename",
                                                     "/media/mpantic/24AFF60273D9FBF4/processed/scan_2.pts");
  frame_name_ = nh_private_.param<std::string>("frame_name", "world");

  if (publish_on_start_) {
    if (true) {
      ROS_INFO_STREAM("Published Mesh on Start");
    } else {
      ROS_WARN_STREAM("Could not publish Mesh on Start");
    }
  }

  createMeshModel();
}

bool LeicaScanPublisher::createMeshModel() {
  // read in pts file
  std::ifstream in_file(default_filename_);
  if (!in_file.is_open()) {
    return false;
  }

  // get length (first line)
  int length;
  in_file >> length;
  ROS_INFO_STREAM("Reading " << length << " points");

  cad_percept::meshing::InputPointCloud::Ptr points_orig =
      boost::make_shared<cad_percept::meshing::InputPointCloud>();
  cad_percept::meshing::InputPointCloud::Ptr points =
      boost::make_shared<cad_percept::meshing::InputPointCloud>();
  cad_percept::meshing::InputNormals::Ptr normals =
      boost::make_shared<cad_percept::meshing::InputNormals>();
  cad_percept::meshing::InputPointCloud::Ptr filteredpoints =
      boost::make_shared<cad_percept::meshing::InputPointCloud>();

  Eigen::Matrix3d rotation;
  rotation.row(0) << 1, 0, 0;
  rotation.row(1) << 0, 0, 1;
  rotation.row(2) << 0, -1, 0;
  Eigen::Affine3d meshing_transform(rotation);

  // todo(mpantic): no input validation. Bad.
  struct LeicaFileLine leica_line;
  while (in_file >> leica_line.x >> leica_line.y >> leica_line.z >> leica_line.snr >>
         leica_line.r >> leica_line.g >> leica_line.b) {
    Eigen::Vector3d point(leica_line.x, leica_line.y, leica_line.z);
    auto pt_transformed = meshing_transform * point;
    points->push_back({static_cast<float>(pt_transformed.x()),
                       static_cast<float>(pt_transformed.y()),
                       static_cast<float>(pt_transformed.z())});

    points_orig->push_back({static_cast<float>(point.x()),
                            static_cast<float>(point.y()),
                            static_cast<float>(point.z())});
  }

  PreProcessingFilter filter;
  filter.addNeighborhoodFilter(30, 1.0);
  filter.addVoxelFilter(0.02, 0.02, 0.02);
  filter.run(points, filteredpoints, normals);


  // create teh mesh
  Delaunay2DMesher::Mesher mesher;
  mesher.addPointCloud(filteredpoints, normals);

  std::cout << points->size() << std::endl;
  cad_percept::cgal::Polyhedron mesh;
  mesher.getMesh(&mesh);

  std::cout << meshing_transform.matrix() << std::endl;
  // restore original coordinates
  for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
    cgal::Point pt = it->point();
    Eigen::Vector3d eig_pt(pt.x(), pt.y(), pt.z());
    Eigen::Vector3d eig_pt_transf;
    eig_pt_transf = meshing_transform.inverse() * eig_pt;
    it->point() = cgal::Point(eig_pt_transf.x(), eig_pt_transf.y(), eig_pt_transf.z());
  }

  // publish mesh
  cgal_msgs::TriangleMeshStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  cgal::triangleMeshToMsg(mesh, &msg.mesh);
  pub_mesh_.publish(msg);
  // skip first line
  points_orig->header.frame_id = "map";
  pub_pcl_.publish(points_orig h);

  return true;
}

}  // namespace meshing
}  // namespace cad_percept

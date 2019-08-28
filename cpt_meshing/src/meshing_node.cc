#include <cpt_meshing/meshing_node.h>

namespace cad_percept {
namespace meshing {

CadPerceptMeshingNode::CadPerceptMeshingNode(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // only one meshing type at the moment.
  mesher_ = std::make_shared<Delaunay2DMesher::Mesher>();

  // set up box filter if needed.
  if (nh_private_.param<bool>("box_filter/enable", false)) {
    double min, max;
    if (nh_private_.getParam("box_filter/x_min", min) &&
        nh_private_.getParam("box_filter/x_max", max)) {
      ROS_INFO("Adding Box filter on x: [%.2f - %.2f].", min, max);
      preprocessing_.addBoxFilter("x", min, max);
    }

    if (nh_private_.getParam("box_filter/y_min", min) &&
        nh_private_.getParam("box_filter/y_max", max)) {
      ROS_INFO("Adding Box filter on y: [%.2f - %.2f].", min, max);
      preprocessing_.addBoxFilter("y", min, max);
    }

    if (nh_private_.getParam("box_filter/z_min", min) &&
        nh_private_.getParam("box_filter/z_max", max)) {
      ROS_INFO("Adding Box filter on z: [%.2f - %.2f].", min, max);
      preprocessing_.addBoxFilter("z", min, max);
    }
  }

  // set up voxel filter if needed.
  if (nh_private_.param<bool>("voxel_filter/enable", true)) {
    double voxel_x = nh_private_.param<double>("voxel_filter/x_size", 0.02);
    double voxel_y = nh_private_.param<double>("voxel_filter/y_size", 0.02);
    double voxel_z = nh_private_.param<double>("voxel_filter/z_size", 0.02);
    ROS_INFO("Adding Voxel filter with size [%.2f, %.2f, %.2f].", voxel_x, voxel_y, voxel_z);
    preprocessing_.addVoxelFilter(voxel_x, voxel_y, voxel_z);
  }

  preprocessing_.setNormalEstimationRadius(
      nh_private_.param<double>("normal_estimation/radius", 0.03));
  sub_points_ = nh_.subscribe("points", 1, &CadPerceptMeshingNode::pointCoudCallback, this);

  pub_mesh_ = nh.advertise<cgal_msgs::TriangleMeshStamped>("mesh", 1);
}

void CadPerceptMeshingNode::pointCoudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  cad_percept::meshing::InputPointCloud::Ptr input_cloud =
      boost::make_shared<cad_percept::meshing::InputPointCloud>();

  // convert input point cloud
  pcl::PCLPointCloud2 pcl_pointcloud2;
  pcl_conversions::toPCL(*cloud_msg, pcl_pointcloud2);
  pcl::fromPCLPointCloud2(pcl_pointcloud2, *input_cloud);

  // create datastructures
  cad_percept::meshing::InputPointCloud::Ptr cloud_filtered =
      boost::make_shared<cad_percept::meshing::InputPointCloud>();
  cad_percept::meshing::InputNormals::Ptr normals =
      boost::make_shared<cad_percept::meshing::InputNormals>();

  cad_percept::cgal::Polyhedron mesh;

  // run through processing
  preprocessing_.run(input_cloud, cloud_filtered, normals);
  MeshPerformanceCounters perf_count;
  mesher_->addPointCloud(cloud_filtered, normals);
  mesher_->getMesh(&mesh, &perf_count);

  ROS_INFO_STREAM("[Meshing Stats] " << perf_count);

  // publish mesh
  cgal_msgs::TriangleMeshStamped msg;
  msg.header = cloud_msg->header;
  cgal::triangleMeshToMsg(mesh, &msg.mesh);
  pub_mesh_.publish(msg);
}
}
}
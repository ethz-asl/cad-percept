#include <cpt_meshing/mesher/delaunay_3d_mesher.h>
#include <cpt_meshing/mesher_interface.h>
#include <cpt_meshing/preprocessing/pre_processing_filter.h>
#include <cpt_meshing/pcl_typedefs.h>
#include <cgal_msgs/TriangleMesh.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_conversions/mesh_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <iostream>
using namespace std;

namespace cad_percept {
namespace meshing {

class CadPerceptMeshingNode {
 public:
  CadPerceptMeshingNode(ros::NodeHandle nh) :
      nh_(nh) {
    mesher_ = std::make_shared<Delaunay3DMesher>();

    // add some filtering
    preprocessing_.addVoxelFilter(0.02, 0.02, 0.02);
    preprocessing_.addBoxFilter("x", -2.0, 2.0);
    preprocessing_.addBoxFilter("y", -2.0, 2.0);
    preprocessing_.addBoxFilter("z", 0.2, 4.0);

    sub_points_ = nh_.subscribe("points",
                                1,
                                &CadPerceptMeshingNode::pointCoudCallback,
                                this);

    pub_mesh_ = nh.advertise<cgal_msgs::TriangleMesh>("mesh", 1);
  }

  void pointCoudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

    cad_percept::meshing::InputPointCloud::Ptr input_cloud;

    // convert input point cloud
    pcl::PCLPointCloud2 pcl_pointcloud2;
    pcl_conversions::toPCL(*cloud_msg, pcl_pointcloud2);
    pcl::fromPCLPointCloud2(pcl_pointcloud2, *input_cloud);

    // create datastructures
    cad_percept::meshing::InputPointCloud::Ptr cloud_filtered;
    cad_percept::meshing::InputNormals::Ptr normals;
    cad_percept::cgal::Polyhedron mesh;

    // run through processing
    preprocessing_.run(input_cloud, cloud_filtered, normals);
    mesher_->addPointCloud(cloud_filtered, normals);
    mesher_->getMesh(&mesh);

    // publish mesh
    cgal_msgs::TriangleMesh msg;
    cgal::triangleMeshToMsg(mesh, &msg);
    pub_mesh_.publish(msg);
  }

 private:
  PreProcessingFilter preprocessing_;
  std::shared_ptr<MesherInterface> mesher_;

  // ros stuff
  ros::Subscriber sub_points_;
  ros::Publisher pub_mesh_;
  ros::NodeHandle nh_;

};
}
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "cpt_meshing_node");
  ros::NodeHandle nh;

  cad_percept::meshing::CadPerceptMeshingNode meshing_node(nh);

  ros::spin();

  return 0;
}
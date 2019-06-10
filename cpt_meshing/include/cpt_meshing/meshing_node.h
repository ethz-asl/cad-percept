//
// Created by mpantic on 10.06.19.
//

#ifndef CPT_MESHING_MESHING_NODE_H
#define CPT_MESHING_MESHING_NODE_H
#include <cpt_meshing/mesher/delaunay_2d_mesher.h>
#include <cpt_meshing/mesher_interface.h>
#include <cpt_meshing/preprocessing/pre_processing_filter.h>
#include <cpt_meshing/pcl_typedefs.h>
#include <cgal_msgs/TriangleMeshStamped.h>
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
  CadPerceptMeshingNode(ros::NodeHandle nh, ros::NodeHandle nh_private);

  void pointCoudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

 private:
  PreProcessingFilter preprocessing_;
  std::shared_ptr<MesherInterface> mesher_;

  // ros stuff
  ros::Subscriber sub_points_;
  ros::Publisher pub_mesh_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

};
}
}
#endif //CPT_MESHING_MESHING_NODE_H

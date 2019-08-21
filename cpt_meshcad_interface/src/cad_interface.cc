#include "cad_interface/cad_interface.h"

namespace cad_percept {
namespace cad_interface {

CadInterface::CadInterface(ros::NodeHandle& n) : nh_(n), iterator_(0) {
  cad_mesh_pub_ = nh_.advertise<cgal_msgs::TriangleMeshStamped>("cad_model", 100);
};

CadInterface::~CadInterface() {};

void CadInterface::load(const std::string &filename) {
  std::cout << "INFO: Loading CAD file from " << filename << std::endl;
  // Loading of mesh data.
  std::ifstream off_file(filename.c_str(), std::ios::binary);
  if (!CGAL::read_off(off_file, P_)) {
    std::cerr << "Error: Invalid OFF file" << std::endl;
  }
  if (!P_.is_valid() || P_.empty()) {
    std::cerr << "Error: Invalid facegraph" << std::endl;
  }

  CGAL::Polygon_mesh_processing::stitch_borders(P_); // necessary to remove duplicated vertices

  if (!P_.is_valid() || P_.empty()) {
    std::cerr << "Error: Invalid facegraph" << std::endl;
  }

  cgal_msgs::TriangleMesh t_msg;
  cad_percept::cgal::triangleMeshToMsg(P_, &t_msg);
  cad_mesh_msg_.mesh = t_msg;
  cad_mesh_msg_.header.frame_id = "marker2";
}

void CadInterface::publishPointCloudThread() {
    ros::Rate thread_rate(0.5);
    while (ros::ok()) {

      // Publish
      cad_mesh_msg_.header.stamp = ros::Time::now();
      cad_mesh_msg_.header.seq = iterator_++;
      cad_mesh_pub_.publish(cad_mesh_msg_);

      thread_rate.sleep();
    }
}


} //namespace cad_interface
}

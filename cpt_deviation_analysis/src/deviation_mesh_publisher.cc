#include <cpt_deviation_analysis/deviation_mesh_publisher.h>

namespace cad_percept {
namespace cpt_deviation_analysis {

DeviationMeshPublisher::DeviationMeshPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : MeshPublisher(nh, nh_private) {
  set_references_ = nh_.serviceClient<cpt_selective_icp::References>("/set_ref");
}

bool DeviationMeshPublisher::publishMesh(nlohmann::json &j) {
  if (!j.empty()) {
    // Create a new mesh model.
    if (!cgal::MeshModel::create(j, &mesh_model_)) {
      return false;
    }
  }
  // publish as mesh msg
  cgal_msgs::TriangleMeshStamped mesh_msg;
  mesh_msg.header.stamp = ros::Time::now();
  mesh_msg.header.frame_id = frame_name_;
  cgal::meshModelToMsg(mesh_model_, &mesh_msg.mesh);
  pub_mesh_.publish(mesh_msg);

  // now also set the references
  cpt_selective_icp::References srv;
  for (auto &[key, val] : j["facedata"].items()) {
    if (val.contains("reference") && val["reference"]) {
      srv.request.data.push_back(key);
    }
  }
  return set_references_.call(srv.request, srv.response);
}

bool DeviationMeshPublisher::triggerPublishMesh(cgal_msgs::PublishMesh::Request &req,
                                                cgal_msgs::PublishMesh::Response &res) {
  // read in json file
  nlohmann::json j;
  std::ifstream json_file(req.path.c_str());
  json_file >> j;
  // load mesh from json
  bool result = publishMesh(j);
  res.success = result;
  res.message = result ? "Published Mesh" : "Mesh not published";
  return true;
}

}  // namespace cpt_deviation_analysis
}  // namespace cad_percept

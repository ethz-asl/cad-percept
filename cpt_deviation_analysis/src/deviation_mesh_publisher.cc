#include <cpt_deviation_analysis/deviation_mesh_publisher.h>

namespace cad_percept {
namespace cpt_deviation_analysis {

DeviationMeshPublisher::DeviationMeshPublisher(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : MeshPublisher(nh, nh_private) {
  set_references_ = nh_.serviceClient<cpt_selective_icp::References>("/set_ref");
  set_deviation_target_ = nh_.serviceClient<cgal_msgs::SetDeviationPlane>("/set_deviation_target");
  publish_service_.shutdown();
  publish_service_ =
      nh_private_.advertiseService("publish", &DeviationMeshPublisher::triggerPublishMesh, this);
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
  cpt_selective_icp::References ref_call;
  for (auto &[key, val] : j["facedata"].items()) {
    if (val.contains("reference") && val["reference"]) {
      ref_call.request.data.push_back(key);
    }
  }
  if (!set_references_.call(ref_call.request, ref_call.response)) return false;

  // set the deviation target
  cgal_msgs::SetDeviationPlane deviation_call;
  for (auto &[key, val] : j["facedata"].items()) {
    if (val.contains("selected") && val["selected"]) {
      deviation_call.request.facet_id == key;
    }
  }
  deviation_call.request.task_id = "1";
  return set_deviation_target_.call(deviation_call.request, deviation_call.response);
}

bool DeviationMeshPublisher::triggerPublishMesh(cgal_msgs::PublishMesh::Request &req,
                                                cgal_msgs::PublishMesh::Response &res) {
  // read in json file
  std::string filename = req.path;
  nlohmann::json j;
  if (filename.empty()) {
    filename = default_filename_;
  }
  std::ifstream json_file(filename.c_str());
  json_file >> j;
  // load mesh from json
  bool result = publishMesh(j);
  res.success = result;
  res.message = result ? "Published Mesh" : "Mesh not published";
  return true;
}

}  // namespace cpt_deviation_analysis
}  // namespace cad_percept

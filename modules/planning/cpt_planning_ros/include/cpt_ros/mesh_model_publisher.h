
#ifndef CPT_ROS_INCLUDE_CPT_ROS_MESH_MODEL_PUBLISHER_H_
#define CPT_ROS_INCLUDE_CPT_ROS_MESH_MODEL_PUBLISHER_H_
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/mesh_model.h>
#include <ros/ros.h>
#include <cgal_msgs/TriangleMeshStamped.h>
namespace cad_percept {

class MeshModelPublisher {
 public:
  MeshModelPublisher(){}

  MeshModelPublisher(ros::NodeHandle& nh, const std::string& topic_name)
      : nh_(nh), topic_name_(topic_name) {
    mesh_publisher_ = nh_.advertise<cgal_msgs::TriangleMeshStamped>(topic_name_, 1, true);
  }

  void publish(cgal::MeshModel::Ptr model, std::string frame="world") {
    cgal_msgs::TriangleMeshStamped msg;
    cgal::Polyhedron poly(model->getMesh());
    cgal::triangleMeshToMsg(poly, &msg.mesh);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";

    if(frame == "uv"){
      for(auto& vertex : msg.mesh.vertices){
        vertex.x += 2.0;
      }
    }
    mesh_publisher_.publish(msg);
  }

  ros::NodeHandle nh_;
  ros::Publisher mesh_publisher_;
  std::string topic_name_;
};
}  // namespace cad_percept
#endif  // CPT_ROS_INCLUDE_CPT_ROS_MESH_MODEL_PUBLISHER_H_

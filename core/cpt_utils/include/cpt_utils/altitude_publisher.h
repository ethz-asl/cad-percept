#ifndef CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_
#define CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_

#include <cgal_conversions/eigen_conversions.h>
#include <cgal_conversions/mesh_conversions.h>
#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/FacetID.h>
#include <cgal_msgs/PublishMesh.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace cad_percept {
namespace cpt_utils {

/*
 * Simple node that reads off files and publishes them as TriangleMeshStamped.
 */
class AltitudePublisher {
 public:
  AltitudePublisher(ros::NodeHandle nh, ros::NodeHandle nh_private) {
    std::string mesh_path;
    nh_private.getParam("mesh_path", mesh_path);
    pub_intersection_ = nh_private.advertise<geometry_msgs::PointStamped>("intersection", 1);
    sub_odom_ = nh_private.subscribe("odometry", 1, &AltitudePublisher::odomCallback, this);
    if (!cgal::MeshModel::create(mesh_path, &mesh_model_)) {
      ROS_FATAL("COULD NOT LOAD MESH MODEL!");
    }
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    cgal::Vector direction(0, 0, -1);  // downward ray
    cgal::Point position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                         msg->pose.pose.position.z);  // downward ray
    cgal::Ray ray(position, direction);

    if (!mesh_model_->isIntersection(ray)) {
      return;
    }

    cgal::Point inter_point = mesh_model_->getIntersection(ray).intersected_point;
    geometry_msgs::PointStamped msg_out;
    msg_out.header.stamp = msg->header.stamp;
    msg_out.header.frame_id = msg->header.frame_id;
    msg_out.point.x = 0;
    msg_out.point.y = 0;
    msg_out.point.z = msg->pose.pose.position.z - inter_point.z();  // output distance
    pub_intersection_.publish(msg_out);
  }

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_odom_;
  ros::Publisher pub_intersection_;

  cgal::MeshModel::Ptr mesh_model_;
};
}  // namespace cpt_utils
}  // namespace cad_percept
#endif  // CPT_UTILS_INCLUDE_CPT_UTILS_MESH_PUBLISHER_H_

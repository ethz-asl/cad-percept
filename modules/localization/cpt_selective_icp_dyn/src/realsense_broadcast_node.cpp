#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// parameters
std::string odom_topic;
std::string realsense_odom_topic;
int input_queue_size;
std::string tf_map_frame;
std::string camera_pose_frame;
std::string camera_odom_frame;
double realsense_cov;

class Realsense
{
  private:
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Subscriber odom_sub_;
    ros::Publisher realsense_odom_pub_;
    bool initialized;
    double init_x;
    double init_y;
    double init_z;

    // Callback function for correcting, publishing and broadcasting the odometry message from Realsense
    void gotOdom(const nav_msgs::Odometry &odom_msg_in){

      std::string child_frame_id = odom_msg_in.child_frame_id;
      std::string parent_frame_id = odom_msg_in.header.frame_id;
      
      geometry_msgs::Pose pose = odom_msg_in.pose.pose;
      geometry_msgs::Point position = pose.position;
      geometry_msgs::Quaternion orientation = pose.orientation;
      geometry_msgs::Twist twist = odom_msg_in.twist.twist;

      nav_msgs::Odometry odom_msg_out = odom_msg_in;

      // Initialize positions to zero
      if (initialized == false){
        init_x = -position.y;
        init_y = position.x;
        init_z = position.z;
        initialized = true;
      }

      // Rotate translation by 90 degrees yaw, orientation is not used anyway except for yaw, which is still correct
      odom_msg_out.pose.pose.position.x = -position.y - init_x;
      odom_msg_out.pose.pose.position.y = position.x - init_y;
      odom_msg_out.pose.pose.position.z = position.z - init_z;
      odom_msg_out.twist.twist.linear.x = -twist.linear.y;
      odom_msg_out.twist.twist.linear.y = twist.linear.x;
      odom_msg_out.twist.twist.linear.z = twist.linear.z;

      // Set covariances from parameters
      odom_msg_out.pose.covariance[0+0*6] = realsense_cov;
      odom_msg_out.pose.covariance[1+1*6] = realsense_cov;
      odom_msg_out.pose.covariance[2+2*6] = realsense_cov;
      odom_msg_out.pose.covariance[3+3*6] = realsense_cov;
      odom_msg_out.pose.covariance[4+4*6] = realsense_cov;
      odom_msg_out.pose.covariance[5+5*6] = realsense_cov;
      odom_msg_out.twist.covariance[0+0*6] = realsense_cov;
      odom_msg_out.twist.covariance[1+1*6] = realsense_cov;
      odom_msg_out.twist.covariance[2+2*6] = realsense_cov;
      odom_msg_out.twist.covariance[3+3*6] = realsense_cov;
      odom_msg_out.twist.covariance[4+4*6] = realsense_cov;
      odom_msg_out.twist.covariance[5+5*6] = realsense_cov;
      
      // publish corrected realsense odom to topic (realsense_odom) for EKF
      if (realsense_odom_pub_.getNumSubscribers()) {
        realsense_odom_pub_.publish(odom_msg_out);
      }

      // broadcast odom to pose to tf 
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(-position.y - init_x, position.x - init_y, position.z - init_z));
      transform.setRotation( tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, odom_msg_in.header.stamp, camera_odom_frame, camera_pose_frame));
      ROS_DEBUG("Broadcasted and published Odom->Pose Transform"); 
    };

  public:
    Realsense(ros::NodeHandle& nh_)
    {
      odom_sub_ = nh_.subscribe(odom_topic, 1, &Realsense::gotOdom, this);
      realsense_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(realsense_odom_topic, 50, true);
      initialized = false;
    };
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "realsense_broadcast");
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private("~");

  // Parameters
  ros::NodeHandle("~").getParam("odomTopic", odom_topic);
  ros::NodeHandle("~").getParam("realsenseOdomTopic", realsense_odom_topic);
  ros::NodeHandle("~").getParam("inputQueueSize", input_queue_size);
  ros::NodeHandle("~").getParam("tfMapFrame", tf_map_frame);
  ros::NodeHandle("~").getParam("cameraPoseFrame", camera_pose_frame);
  ros::NodeHandle("~").getParam("cameraOdomFrame", camera_odom_frame);
  ros::NodeHandle("~").getParam("realsenseCov", realsense_cov);

  Realsense realsense(nh_);

  std::cout << "Wait for start-up" << std::endl;
  sleep(5);  // wait to set up stuff
  std::cout << "Ready!" << std::endl;
  ros::spin();

  return 0;
};
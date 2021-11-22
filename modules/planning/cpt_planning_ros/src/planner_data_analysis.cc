#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Dense>

class DataAnalysis
{
public:
    DataAnalysis(ros::NodeHandle nh):nh_(nh){};
    void dataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

private:
    ros::NodeHandle nh_;
    ros::Publisher goal_follow_acc_pub = nh_.advertise<std_msgs::Float32>("goal_following_acc", 1);
    ros::Publisher straight_force_acc_pub = nh_.advertise<std_msgs::Float32>("straight_forcing_acc", 1);
    
    ros::Publisher drone_collision_avoid_acc_pub = nh_.advertise<std_msgs::Float32>("drone_collision_avoid_acc", 1);
    ros::Publisher rope_collision_avoid_acc_pub = nh_.advertise<std_msgs::Float32>("rope_collision_avoid_acc", 1);

    ros::Publisher alpha_eta_acc_pub = nh_.advertise<std_msgs::Float32>("alpha_eta_acc", 1);
    ros::Publisher damper_acc_pub = nh_.advertise<std_msgs::Float32>("damper_acc", 1);
    ros::Publisher booster_acc_pub = nh_.advertise<std_msgs::Float32>("booster_acc", 1);
    ros::Publisher total_acc_pub = nh_.advertise<std_msgs::Float32>("total_acc", 1);

    ros::Publisher switch_eta_pub = nh_.advertise<std_msgs::Float32>("energy_regulation_switch", 1);
    ros::Publisher switch_beta_pub = nh_.advertise<std_msgs::Float32>("damper_switch", 1);
    ros::Publisher total_vel_pub = nh_.advertise<std_msgs::Float32>("total_vel", 1);


    std_msgs::Float32 abc;
};


/**
 * Visualize the planner data
 * policy acc, vel, etc.
 * use rqt matplot to vis in real time
 */
void DataAnalysis::dataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  int size = msg->data.size();
//   std::cout<<  msg->data.at(size-1) <<std::endl;
//   ROS_INFO("I heard: -1 [%f]", msg->data.at(size-1));
//   ROS_INFO("I heard: -3 [%f]", msg->data.at(size-3));
//   ROS_INFO("I heard: -5 [%f]", msg->data.at(size-5));

  std_msgs::Float32 goal_follow_var;
  goal_follow_var.data = msg->data.at(0);
  goal_follow_acc_pub.publish(goal_follow_var);

  std_msgs::Float32 straight_force_var;
  straight_force_var.data = msg->data.at(7);
  straight_force_acc_pub.publish(straight_force_var);


  if(msg->data.at(size-12)>0){//drone obs avoid enabled
    std_msgs::Float32 drone_avoid_var;
    drone_avoid_var.data = msg->data.at(21)*msg->data.at(25); //M*pi
    drone_collision_avoid_acc_pub.publish(drone_avoid_var);
  }else{
    std_msgs::Float32 drone_avoid_var;
    drone_avoid_var.data = 0.0; //M*pi
    drone_collision_avoid_acc_pub.publish(drone_avoid_var);
  }

  if(msg->data.at(size-9)>0){//rope avoid policies
    Eigen::Vector3d rope_acc_vec = Eigen::Vector3d::Zero();
    int idx = (int) (msg->data.at(size-10)
                + msg->data.at(size-11)
                + msg->data.at(size-12)
                + msg->data.at(size-13));
    std::cout <<"policy class:"<<msg->data.at(size-13)<<"; "<<msg->data.at(size-12)<<"; "<<
                msg->data.at(size-11)<<"; "<<msg->data.at(size-10)<<"; "<<msg->data.at(size-9)<<std::endl;
    for(int i = 0; i<(int)(msg->data.at(size-9)); i++){
        Eigen::Vector3d rope_node_acc;
        int idx_1 = idx*7+i*7+1;
        int idx_2 = idx*7+i*7+2;
        int idx_3 = idx*7+i*7+3;
        int idx_4 = idx*7+i*7+4;
        int idx_5 = idx*7+i*7+5;
        int idx_6 = idx*7+i*7+6;

        std::cout << idx_1 << std::endl;
        rope_node_acc.x() = msg->data.at(idx_1)*msg->data.at(idx_4);
        rope_node_acc.y() = msg->data.at(idx_2)*msg->data.at(idx_5);
        rope_node_acc.z() = msg->data.at(idx_3)*msg->data.at(idx_6);
        
        rope_acc_vec += rope_node_acc;
    }
    std_msgs::Float32 rope_avoid_var;
    rope_avoid_var.data = rope_acc_vec.norm();
    rope_collision_avoid_acc_pub.publish(rope_avoid_var);
  }else{
    std_msgs::Float32 rope_avoid_var;
    rope_avoid_var.data = 0.0;
    rope_collision_avoid_acc_pub.publish(rope_avoid_var);
  }

  std_msgs::Float32 alpha_eta_acc_var;
  alpha_eta_acc_var.data = msg->data.at(size-7);
  alpha_eta_acc_pub.publish(alpha_eta_acc_var);

  std_msgs::Float32 switch_eta_var;
  switch_eta_var.data = msg->data.at(size-6);
  switch_eta_pub.publish(switch_eta_var);

  std_msgs::Float32 damper_acc_var;
  damper_acc_var.data = msg->data.at(size-5);
  damper_acc_pub.publish(damper_acc_var);

  std_msgs::Float32 switch_beta_var;
  switch_beta_var.data = msg->data.at(size-4);
  switch_beta_pub.publish(switch_beta_var);

  std_msgs::Float32 booster_acc_var;
  booster_acc_var.data = msg->data.at(size-3);
  booster_acc_pub.publish(booster_acc_var);

  std_msgs::Float32 total_vel_var;
  total_vel_var.data = msg->data.at(size-2);
  total_vel_pub.publish(total_vel_var);

  std_msgs::Float32 total_acc_var;
  total_acc_var.data = msg->data.at(size-1);
  total_acc_pub.publish(total_acc_var);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_data");
  ros::NodeHandle n;

  DataAnalysis dataAnalysis(n);
  ros::Subscriber sub = n.subscribe("opt_fabric_planner/policy_vis", 100, &DataAnalysis::dataCallback, &dataAnalysis);
  ROS_INFO("Planner data analysis with recorded rosbag");

  ros::spin();
  return 0;
}
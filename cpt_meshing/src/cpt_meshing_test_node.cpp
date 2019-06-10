#include <cpt_meshing/meshing_node.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
using namespace std;

void createPublishPointCloud(cad_percept::meshing::CadPerceptMeshingNode& node,
                             ros::Publisher& pub) {

  // Create a Pointcloud message
  sensor_msgs::PointCloud2Ptr
      cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();

  // set header
  cloud_msg->header.frame_id = "world";
  cloud_msg->header.stamp = ros::Time::now();
  cloud_msg->width = 10;
  cloud_msg->height = 10;
  cloud_msg->point_step = sizeof(float) * 3;
  cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
  cloud_msg->is_dense = true;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

  // generate points in xyz
  for (unsigned int i = 0; i < 10; i++) {
    for (unsigned int j = 0; j < 10; j++, ++iter_x, ++iter_y, ++iter_z) {
      *iter_x = (float) i;
      *iter_y = (float) j;
      *iter_z = 1.0;
    }
  }
  pub.publish(cloud_msg);
  node.pointCoudCallback(cloud_msg);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "cpt_meshing_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  cad_percept::meshing::CadPerceptMeshingNode meshing_node(nh, nh_private);
  ros::Publisher pub_pcl;
  pub_pcl = nh.advertise<sensor_msgs::PointCloud2>("points_test", 1);

  while (true) {
    createPublishPointCloud(meshing_node, pub_pcl);
    ros::spinOnce();
    std::cin.get();
  }
  return 0;
}
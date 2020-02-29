#include <pointmatcher_ros/point_cloud.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  rosbag::Bag bag;
  std::cout << "Data Reader started" << std::endl;

  bag.open("../../../smb_2020-01-29-14-58-07.bag", rosbag::bagmode::Read);
  ros::Publisher scan_pub_ = nh.advertise<sensor_msgs::PointCloud2>("lidar_frame", 1, true);

  std::vector<std::string> topics;
  topics.push_back(std::string("/rslidar_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::string savefolder = "../../../data_set/scan";
  std::string scan_name;
  std::string filetype = ".bag";
  int scan_nr = 0;
  rosbag::Bag bagwrite;

  boost::filesystem::remove_all("../../../data_set");
  boost::filesystem::create_directory("../../../data_set");
  std::cout << "Cleared data folder" << std::endl;

  for (rosbag::MessageInstance const m : view) {
    sensor_msgs::PointCloud2::ConstPtr actuel = m.instantiate<sensor_msgs::PointCloud2>();

    if (actuel != NULL) {
      scan_pub_.publish(actuel);

      // Create new bag file
      scan_name = savefolder + std::to_string(scan_nr) + filetype;
      std::ofstream actuel_file(scan_name);
      actuel_file << "" << std::endl;
      actuel_file.close();

      bagwrite.open(scan_name, rosbag::bagmode::Write);

      bagwrite.write("/rslidar_points", ros::Time::now(), *actuel);

      bagwrite.close();
      std::cout << "Created scan nr. " << std::to_string(scan_nr) << std::endl;

      scan_nr++;
    }
  }
  bag.close();

  return 0;
}

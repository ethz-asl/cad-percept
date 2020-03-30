#include <geometry_msgs/PointStamped.h>
#include <pointmatcher_ros/point_cloud.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <fstream>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  rosbag::Bag bag;
  std::cout << "Data Reader started" << std::endl;

  std::string raw_file = nh_private.param<std::string>("raw_data", "fail");
  bag.open(raw_file, rosbag::bagmode::Read);

  std::vector<std::string> lidar_topic;
  lidar_topic.push_back(nh_private.param<std::string>("point_topic", "fail"));
  rosbag::View view_lidar(bag, rosbag::TopicQuery(lidar_topic));
  std::cout << "Created lidar frame view" << std::endl;

  std::vector<std::string> gt_topic;
  gt_topic.push_back(nh_private.param<std::string>("gt_topic", "fail"));
  rosbag::View view_gt(bag, rosbag::TopicQuery(gt_topic));
  std::cout << "Created ground truth view" << std::endl;

  // Setups for saved files
  std::string save_folder = nh_private.param<std::string>("save_folder", "");
  std::string file_name;
  std::string file_type = ".bag";
  std::string scan_name = "/scan";
  int scan_nr = 0;
  bool gt_found = false;
  rosbag::Bag bag_write;

  boost::filesystem::remove_all(save_folder);
  boost::filesystem::create_directory(save_folder);
  std::cout << "Cleared data folder" << std::endl;

  for (rosbag::MessageInstance const mlidar : view_lidar) {
    sensor_msgs::PointCloud2::Ptr lidar_scan = mlidar.instantiate<sensor_msgs::PointCloud2>();
    std::cout << "Got new lidar frame" << std::endl;

    // Search for (one of) the corresponding ground truth data at same time stamp
    gt_found = false;
    for (rosbag::MessageInstance const mgt : view_gt) {
      geometry_msgs::PointStamped::Ptr ground_truth =
          mgt.instantiate<geometry_msgs::PointStamped>();

      if (lidar_scan != NULL && ground_truth != NULL &&
          abs((int)(lidar_scan->header.stamp.sec - ground_truth->header.stamp.sec)) == 0) {
        // Valid lidarframe and ground truth pair found
        std::cout << "Found valid lidar and ground truth data pair" << std::endl;
        file_name = save_folder + scan_name + std::to_string(scan_nr) + file_type;
        std::ofstream actuel_file(file_name);
        actuel_file << "" << std::endl;
        actuel_file.close();

        bag_write.open(file_name, rosbag::bagmode::Write);

        bag_write.write("/rslidar_points", ros::Time::now(), *lidar_scan);
        bag_write.write("/ground_truth", ros::Time::now(), *ground_truth);

        bag_write.close();
        std::cout << "Created scan nr. " << std::to_string(scan_nr) << std::endl;

        scan_nr++;
        gt_found = true;

        break;  // Stop searching for this lidar frame if one ground truth found

      } else if (lidar_scan == NULL && ground_truth == NULL) {
        std::cout << "no lidar frame and ground_truth found" << std::endl;
      } else if (lidar_scan == NULL) {
        std::cout << "no lidar frame found" << std::endl;
      } else if (ground_truth == NULL) {
        std::cout << "no ground truth found" << std::endl;
      }
    }
    if (!gt_found) {
      std::cout << "but no ground truth with same timestemp found, drop this lidar frame"
                << std::endl;
    }
  }
  bag.close();
  return 0;
}
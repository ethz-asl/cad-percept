#include <geometry_msgs/PointStamped.h>
#include <pointmatcher_ros/point_cloud.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/filesystem.hpp>

// The idea of this program is to create smaller bag files including one corresponding
// frame and one ground thruth data, such that one has direct access to the single frames.
int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  rosbag::Bag bag;
  std::cout << "Data Reader started" << std::endl;

  bag.open("../../../smb_2020-01-29-14-58-07.bag", rosbag::bagmode::Read);

  std::vector<std::string> lidartopic;
  lidartopic.push_back(std::string("/rslidar_points"));
  rosbag::View viewlidar(bag, rosbag::TopicQuery(lidartopic));
  std::cout << "Created lidar frame view" << std::endl;

  std::vector<std::string> gttopic;
  gttopic.push_back(std::string("/leica/position"));
  rosbag::View viewgt(bag, rosbag::TopicQuery(gttopic));
  std::cout << "Created ground truth view" << std::endl;

  std::string savefolder = "../../../data_set/scan";
  std::string scan_name;
  std::string filetype = ".bag";
  int scan_nr = 0;
  bool gtfound = false;
  rosbag::Bag bagwrite;

  boost::filesystem::remove_all("../../../data_set");
  boost::filesystem::create_directory("../../../data_set");
  std::cout << "Cleared data folder" << std::endl;

  for (rosbag::MessageInstance const mlidar : viewlidar) {
    sensor_msgs::PointCloud2::Ptr lidarframe = mlidar.instantiate<sensor_msgs::PointCloud2>();
    std::cout << "Got new lidar frame" << std::endl;

    // Search for (one of) the corresponding ground truth data at same time stamp
    gtfound = false;
    for (rosbag::MessageInstance const mgt : viewgt) {
      geometry_msgs::PointStamped::Ptr ground_truth =
          mgt.instantiate<geometry_msgs::PointStamped>();

      if (lidarframe != NULL && ground_truth != NULL &&
          abs((int)(lidarframe->header.stamp.sec - ground_truth->header.stamp.sec)) == 0) {
        // Valid lidarframe and ground truth pair found
        std::cout << "Found valid lidar and ground truth data pair" << std::endl;
        scan_name = savefolder + std::to_string(scan_nr) + filetype;
        std::ofstream actuel_file(scan_name);
        actuel_file << "" << std::endl;
        actuel_file.close();

        bagwrite.open(scan_name, rosbag::bagmode::Write);

        bagwrite.write("/rslidar_points", ros::Time::now(), *lidarframe);
        bagwrite.write("/ground_truth", ros::Time::now(), *ground_truth);

        bagwrite.close();
        std::cout << "Created scan nr. " << std::to_string(scan_nr) << std::endl;

        scan_nr++;
        gtfound = true;

        break;  // Stop searching for this lidar frame if one ground truth found

      } else if (lidarframe == NULL && ground_truth == NULL) {
        std::cout << "no lidar frame and ground_truth found" << std::endl;
      } else if (lidarframe == NULL) {
        std::cout << "no lidar frame found" << std::endl;
      } else if (ground_truth == NULL) {
        std::cout << "no ground truth found" << std::endl;
      }
    }
    if (!gtfound) {
      std::cout << "but no ground truth with same timestemp found, drop this lidar frame"
                << std::endl;
    }
  }
  bag.close();
  return 0;
}

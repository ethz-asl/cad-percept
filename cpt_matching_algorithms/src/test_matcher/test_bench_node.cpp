#include <ros/ros.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <chrono>
#include <random>

#include "plane_extraction/plane_extraction.h"
#include "plane_matching/plane_matching.h"
#include "test_matcher/map_plane_extractor.h"

using namespace cad_percept;
using namespace matching_algorithms;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

pcl::PointCloud<pcl::PointXYZ> lidar_scan;
std::vector<float> bin_elevation;
float noise_variance;
float range_of_lidar;
float dtheta;

ros::Publisher scan_pub;
ros::Subscriber map_sub;
cad_percept::cgal::MeshModel::Ptr reference_mesh;
bool map_ready = false;
std::string tf_map_frame = "/map";
std::string tf_lidar_frame = "marker_pose";

Eigen::Vector3d gt_translation;
Eigen::Quaterniond gt_rotation;
float transform_TR_[7] = {0, 0, 0, 0, 0, 0, 0};
cad_percept::cgal::Transformation ctransformation;

MapPlanes *map_planes;

void samplePose();
void simulateLidar(cad_percept::cgal::Transformation ctransformation,
                   cad_percept::cgal::MeshModel mesh);
void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);
void runTestIterations();

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_bench");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::cout << "/// Started test bench ///" << std::endl;

  range_of_lidar = nh_private.param<float>("rangeOfLidar", 20);
  bin_elevation = nh_private.param<std::vector<float>>("binElevation", {0});
  dtheta = nh_private.param<float>("lidarAngularResolution", 1);
  noise_variance = nh_private.param<float>("accuracyOfLidar", 0.02);

  map_sub = nh.subscribe("/mesh_publisher/mesh_out", 1, &getCAD);
  scan_pub = nh.advertise<sensor_msgs::PointCloud2>("matched_point_cloud", 1);
  std::cout << "Tab Enter to start tests" << std::endl;
  std::cin.ignore();

  ros::spin();

  return 0;
}

// Get CAD and sample points
void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
  if (!map_ready) {
    ros::NodeHandle nh_private("~");

    std::cout << "Processing CAD mesh" << std::endl;
    std::string frame_id = cad_mesh_in.header.frame_id;
    cad_percept::cgal::msgToMeshModel(cad_mesh_in.mesh, &reference_mesh);

    // Get transformation from /map to mesh
    tf::StampedTransform transform;
    tf::TransformListener tf_listener(ros::Duration(30));
    try {
      tf_listener.waitForTransform(tf_map_frame, frame_id, ros::Time(0), ros::Duration(5.0));
      tf_listener.lookupTransform(tf_map_frame, frame_id, ros::Time(0),
                                  transform);  // get transformation at latest time T_map_to_frame
    } catch (tf::TransformException ex) {
      ROS_ERROR_STREAM("Couldn't find transformation to mesh system");
    }

    cgal::tfTransformationToCGALTransformation(transform, ctransformation);
    reference_mesh->transform(ctransformation);

    std::string file_name = nh_private.param<std::string>("map_plane_file", "fail");

    // load data from file
    map_planes = new MapPlanes();
    map_planes->loadFromYamlFile(file_name);
    map_planes->dispAllPlanes();

    std::cout << "CAD ready" << std::endl;
    map_ready = true;

    runTestIterations();
  }
}

void runTestIterations() {
  ros::NodeHandle nh_private("~");
  // Test iterations
  int test_iterations = nh_private.param<int>("testIterations", 1);
  bool usetoyexample = nh_private.param<bool>("usetoyexample", true);

  int structure_threshold;
  float search_radius;

  std::string extractor;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_planes;
  std::vector<Eigen::Vector3d> plane_normals;
  ros::Publisher plane_pub_;

  pcl::PointNormal norm_point;
  pcl::PointXYZ plane_centroid;
  pcl::PointCloud<pcl::PointNormal> scan_planes;

  int plane_nr = 0;
  float translation_error = 0;
  float rotation_error = 0;
  std::chrono::steady_clock::time_point t_start;
  std::chrono::steady_clock::time_point t_end;
  std::chrono::duration<int, std::milli> duration;

  Eigen::Matrix4d sample_transform;
  Eigen::Matrix4f res_transform;
  Eigen::Vector3f translation;
  Eigen::Quaternionf q;
  float transform_error;

  std::string test_result_file = nh_private.param<std::string>("test_results", "fail");
  std::string data_set_folder = nh_private.param<std::string>("data_set_folder", "fail");
  std::string lidar_scan_file;
  int start_scan_nr = nh_private.param<int>("start_scan_nr", 0);
  int end_scan_nr = nh_private.param<int>("end_scan_nr", 10);
  int scan_nr = start_scan_nr;
  std::vector<int> nan_indices;

  structure_threshold = nh_private.param<int>("StructureThreshold", 150);
  pcl::PointCloud<pcl::PointXYZI> static_structure_cloud;
  search_radius = nh_private.param<float>("Voxelsearchradius", 0.01);

  extractor = nh_private.param<std::string>("PlaneExtractor", "fail");
  srand(time(0));

  std::vector<std::vector<float>> results;

  std::ofstream actuel_file(test_result_file);

  for (int iter = 0;
       (iter < test_iterations && usetoyexample) || ((scan_nr < end_scan_nr) && !usetoyexample);
       iter++, scan_nr++) {
    std::cout << "Start iteration " << iter << std::endl;

    if (usetoyexample) {
      // Sample pose in mesh
      samplePose();

      sample_transform = Eigen::Matrix4d::Identity();
      sample_transform.block(0, 0, 3, 3) = gt_rotation.matrix();
      sample_transform.block(0, 3, 3, 1) = gt_translation;
      cgal::eigenTransformationToCgalTransformation(sample_transform, &ctransformation);

      // Simulate lidar at position
      simulateLidar(ctransformation, *reference_mesh);
    } else {
      // Load real data
      std::cout << "Load real data" << std::endl;
      std::fstream gt_file(data_set_folder + "/ground_truth_" + std::to_string(scan_nr) + ".txt");
      gt_file >> gt_translation[0] >> gt_translation[1] >> gt_translation[2];
      std::cout << gt_translation[0] << " " << gt_translation[1] << " " << gt_translation[2]
                << std::endl;
      gt_file.close();
      lidar_scan_file = data_set_folder + "/scan_" + std::to_string(scan_nr) + ".pcd";
      pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_scan_file, static_structure_cloud);
      pcl::removeNaNFromPointCloud(static_structure_cloud, static_structure_cloud, nan_indices);
      pcl::copyPointCloud(static_structure_cloud, lidar_scan);
    }

    std::cout << "Start time measurement" << std::endl;

    t_start = std::chrono::steady_clock::now();
    // Detect planes
    // Filtering / Preprocessing Point Cloud
    if (nh_private.param<bool>("useStructureFilter", false)) {
      CloudFilter::filterStaticObject(structure_threshold, lidar_scan, static_structure_cloud);
    }
    if (nh_private.param<bool>("useVoxelCentroidFilter", false)) {
      CloudFilter::filterVoxelCentroid(search_radius, lidar_scan);
    }

    if (lidar_scan.size() == 0) {
      std::cout << "Can not find any planes in scan, as scan is empty after filtering" << std::endl;
      break;
    }
    // Plane Extraction
    if (!extractor.compare("pclPlaneExtraction")) {
      PlaneExtractor::pclPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
                                         tf_lidar_frame, plane_pub_);
    } else if (!extractor.compare("rhtPlaneExtraction")) {
      PlaneExtractor::rhtPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
                                         tf_lidar_frame, plane_pub_);
    } else if (!extractor.compare("iterRhtPlaneExtraction")) {
      PlaneExtractor::iterRhtPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
                                             tf_lidar_frame, plane_pub_);

    } else if (!extractor.compare("cgalRegionGrowing")) {
      PlaneExtractor::cgalRegionGrowing(extracted_planes, plane_normals, lidar_scan, tf_lidar_frame,
                                        plane_pub_);
    } else {
      std::cout << "Error: Could not find given plane extractor" << std::endl;
      break;
    }

    // Convert planes to PointNormal PointCloud
    plane_nr = 0;
    for (auto plane_normal : plane_normals) {
      pcl::computeCentroid(extracted_planes[plane_nr], plane_centroid);
      norm_point.x = plane_centroid.x;
      norm_point.y = plane_centroid.y;
      norm_point.z = plane_centroid.z;

      norm_point.normal_x = plane_normal[0];
      norm_point.normal_y = plane_normal[1];
      norm_point.normal_z = plane_normal[2];

      // Correct normals considering normal direction convention for matching
      if (norm_point.x * norm_point.normal_x + norm_point.y * norm_point.normal_y +
              norm_point.z * norm_point.normal_z <
          0) {
        norm_point.normal_x = -norm_point.normal_x;
        norm_point.normal_y = -norm_point.normal_y;
        norm_point.normal_z = -norm_point.normal_z;
      }

      scan_planes.push_back(norm_point);
      plane_nr++;
    }

    // Match planes
    results.clear();
    transform_error = PlaneMatch::PRRUS(transform_TR_, scan_planes, *map_planes, results);
    t_end = std::chrono::steady_clock::now();

    // for (auto result : results) {
    //   translation_error =
    //       (Eigen::Vector3d(result[0], result[1], result[2]) - gt_translation).norm();
    //   rotation_error =
    //       Eigen::AngleAxis<double>(
    //           Eigen::Quaterniond(result[3], result[4], result[5], result[6]).toRotationMatrix() *
    //           gt_rotation.inverse().toRotationMatrix())
    //           .angle();
    //   std::cout << gt_translation[0] << " " << gt_translation[1] << " " << gt_translation[2] << "
    //   "
    //             << gt_rotation.w() << " " << gt_rotation.x() << " " << gt_rotation.y() << " "
    //             << gt_rotation.z() << " " << result[0] << " " << result[1] << " " << result[2]
    //             << " " << result[3] << " " << result[4] << " " << result[5] << " " << result[6]
    //             << " " << translation_error << " " << rotation_error << " " << result[7]
    //             << std::endl;
    //   actuel_file << gt_translation[0] << " " << gt_translation[1] << " " << gt_translation[2]
    //               << " " << gt_rotation.w() << " " << gt_rotation.x() << " " << gt_rotation.y()
    //               << " " << gt_rotation.z() << " " << result[0] << " " << result[1] << " "
    //               << result[2] << " " << result[3] << " " << result[4] << " " << result[5] << " "
    //               << result[6] << " " << translation_error << " " << rotation_error << " "
    //               << result[7] << std::endl;
    // }

    // Evaluate
    if (transform_TR_[0] == 0 && transform_TR_[1] == 0 && transform_TR_[2] == 0 &&
        transform_TR_[3] == 0 && transform_TR_[4] == 0 && transform_TR_[5] == 0 &&
        transform_TR_[6] == 0) {
      std::cout << "Could not find orthogonal planes" << std::endl;
      std::cout << "ground truth position: x: " << gt_translation[0] << " y: " << gt_translation[1]
                << " z: " << gt_translation[2] << std::endl;

      actuel_file << gt_translation[0] << " " << gt_translation[1] << " " << gt_translation[2]
                  << " " << gt_rotation.w() << " " << gt_rotation.x() << " " << gt_rotation.y()
                  << " " << gt_rotation.z() << " " << transform_TR_[0] << " " << transform_TR_[1]
                  << " " << transform_TR_[2] << " " << transform_TR_[3] << " " << transform_TR_[4]
                  << " " << transform_TR_[5] << " " << transform_TR_[6] << " " << -1 << " " << -1
                  << " " << -1 << std::endl;
      /*
            actuel_file << gt_translation[0] << " " << gt_translation[1] << " " << gt_translation[2]
                        << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << transform_TR_[0]
         << " "
                        << transform_TR_[1] << " " << transform_TR_[2] << " " << transform_TR_[3] <<
         " "
                        << transform_TR_[4] << " " << transform_TR_[5] << " " << transform_TR_[6] <<
         " "
                        << -1 << " " << -1 << " " << -1 << std::endl;
                        */

    } else {
      // // Transform LiDAR frame
      res_transform = Eigen::Matrix4f::Identity();
      Eigen::Vector3f translation =
          Eigen::Vector3f(transform_TR_[0], transform_TR_[1], transform_TR_[2]);
      q = Eigen::Quaternionf(transform_TR_[3], transform_TR_[4], transform_TR_[5],
                             transform_TR_[6]);
      res_transform.block(0, 0, 3, 3) = q.matrix();
      res_transform.block(0, 3, 3, 1) = translation;

      pcl::transformPointCloud(lidar_scan, lidar_scan, res_transform);

      DP ref_dp = cpt_utils::pointCloudToDP(lidar_scan);
      scan_pub.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(ref_dp, tf_map_frame,
                                                                          ros::Time::now()));
      ros::spinOnce();

      std::cout << "calculated position: x: " << transform_TR_[0] << " y: " << transform_TR_[1]
                << " z: " << transform_TR_[2] << std::endl;
      std::cout << "ground truth position: x: " << gt_translation[0] << " y: " << gt_translation[1]
                << " z: " << gt_translation[2] << std::endl;
      std::cout << "calculated orientation: qw: " << transform_TR_[3] << " qx: " << transform_TR_[4]
                << " qy: " << transform_TR_[5] << " qz: " << transform_TR_[6] << std::endl;
      // std::cout << "ground truth orientation: qw: " << gt_rotation.w() << " qx: " <<
      // gt_rotation.x()
      //           << " qy: " << gt_rotation.y() << " qz: " << gt_rotation.z() << std::endl;

      translation_error =
          (Eigen::Vector3d(transform_TR_[0], transform_TR_[1], transform_TR_[2]) - gt_translation)
              .norm();
      rotation_error =
          Eigen::AngleAxis<double>(Eigen::Quaterniond(transform_TR_[3], transform_TR_[4],
                                                      transform_TR_[5], transform_TR_[6])
                                       .toRotationMatrix() *
                                   gt_rotation.inverse().toRotationMatrix())
              .angle();
      duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
      std::cout << "translation error: " << translation_error << std::endl;
      // std::cout << "rotation error: " << rotation_error << std::endl;
      std::cout << "time needed: " << duration.count() << " milliseconds" << std::endl;

      actuel_file << gt_translation[0] << " " << gt_translation[1] << " " << gt_translation[2]
                  << " " << gt_rotation.w() << " " << gt_rotation.x() << " " << gt_rotation.y()
                  << " " << gt_rotation.z() << " " << transform_TR_[0] << " " << transform_TR_[1]
                  << " " << transform_TR_[2] << " " << transform_TR_[3] << " " << transform_TR_[4]
                  << " " << transform_TR_[5] << " " << transform_TR_[6] << " " << translation_error
                  << " " << rotation_error << " " << duration.count() << std::endl;

      /*actuel_file << gt_translation[0] << " " << gt_translation[1] << " " << gt_translation[2]
                  << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << transform_TR_[0] << " "
                  << transform_TR_[1] << " " << transform_TR_[2] << " " << transform_TR_[3] << " "
                  << transform_TR_[4] << " " << transform_TR_[5] << " " << transform_TR_[6] << " "
                  << translation_error << " " << 0 << " " << duration.count() << std::endl;*/
    }

    // Preparation for next iteration
    extracted_planes.clear();
    plane_normals.clear();
    scan_planes.clear();
    transform_TR_[0] = 0;
    transform_TR_[1] = 0;
    transform_TR_[2] = 0;
    transform_TR_[3] = 0;
    transform_TR_[4] = 0;
    transform_TR_[5] = 0;
    transform_TR_[6] = 0;
  }
  actuel_file.close();
}

bool in_map_bit_map_garage[61][15]{
    // -7 -6 -5 -4 -3 -2 -1  0  1  2  3  4  5  6  7
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-23//////////////////////////
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-22//////////////////////////
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-21//////////////////////////
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-20//////////////////////////
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-19//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-18//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-17//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-16//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-15//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-14//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-13//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-12//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-11//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-10//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-9//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-8//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-7//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-6//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-5//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-4//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-3//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-2//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  //-1//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  // 0//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 1//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 2//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 3//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 4//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 5//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 6//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 7//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 8//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 9//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 10//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 11//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 12//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 13//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 14//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 15//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 16//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 17//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 18//////////////////////////
    {0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},  // 19//////////////////////////
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 20//////////////////////////
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 21//////////////////////////
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 22//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 23//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 24//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 25//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 26//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 27//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 28//////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 29//////////////////////////
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 30//////////////////////////
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 31//////////////////////////
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 32//////////////////////////
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 33//////////////////////////
    {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // 34//////////////////////////
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  // 35//////////////////////////
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},  // 36//////////////////////////
    {0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0}   // 37//////////////////////////
};

bool in_map_bit_map_lee_h[12][13]{
    //-9 -8 -7 -6 -5 -4 -3 -2 -1 0 1  2  3
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // -4/////////////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // -3/////////////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // -2/////////////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},  // -1/////////////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  //  0/////////////////////////////////
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0},  //  1/////////////////////////////////
    {1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  //  2/////////////////////////////////
    {1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},  //  3/////////////////////////////////
    {1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0},  //  4/////////////////////////////////
    {1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  //  5/////////////////////////////////
    {1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  //  6/////////////////////////////////
    {1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  //  7/////////////////////////////////

};

void samplePose() {
  bool valid_position = false;
  int x_coord;
  int y_coord;
  while (!valid_position) {
    x_coord = std::rand() % 61;
    y_coord = std::rand() % 15;
    if (in_map_bit_map_lee_h[x_coord][y_coord]) {
      valid_position = true;
    }
  }
  gt_translation[0] = x_coord - 23;
  gt_translation[1] = y_coord - 7;
  gt_translation[2] = (double)(std::rand() % 20) * 0.1 + 0.5;

  // Uniform sampling
  double euler_x = (std::rand() % 30) * M_PI / 15 - M_PI / 2;
  double euler_y = (std::rand() % 30) * M_PI / 15 - M_PI / 2;
  double euler_z = (std::rand() % 30) * M_PI / 15 - M_PI / 2;

  // double euler_x = 0;
  // double euler_y = 0;
  // double euler_z = (std::rand() % 30) * M_PI / 15 - M_PI / 2;

  gt_rotation = Eigen::AngleAxisd(euler_x, Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxisd(euler_y, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(euler_z, Eigen::Vector3d::UnitZ());
};

void simulateLidar(cad_percept::cgal::Transformation ctransformation,
                   cad_percept::cgal::MeshModel mesh) {
  // Transform mesh
  mesh.transform(ctransformation.inverse());
  // Add lidar properties
  // Bin characteristic & visible
  float x_unit;
  float y_unit;
  float z_unit;
  float PI_angle = (float)(M_PI / 180);
  cad_percept::cgal::Ray bin_ray;
  cad_percept::cgal::Point origin(0, 0, 0);
  cad_percept::cgal::Point unit_dir;
  cad_percept::cgal::Intersection inter_point;
  pcl::PointXYZ pcl_inter_point;
  lidar_scan.clear();
  for (auto &bin : bin_elevation) {
    for (float theta = 0; theta < 360; theta += dtheta) {
      x_unit = cos(bin * PI_angle) * cos(theta * PI_angle);
      y_unit = cos(bin * PI_angle) * sin(theta * PI_angle);
      z_unit = sin(bin * PI_angle);
      unit_dir = cad_percept::cgal::Point(x_unit, y_unit, z_unit);
      bin_ray = cad_percept::cgal::Ray(origin, unit_dir);

      if (mesh.isIntersection(bin_ray)) {
        inter_point = mesh.getIntersection(bin_ray);
        pcl_inter_point.x = (float)inter_point.intersected_point.x();
        pcl_inter_point.y = (float)inter_point.intersected_point.y();
        pcl_inter_point.z = (float)inter_point.intersected_point.z();
        lidar_scan.push_back(pcl_inter_point);
      }
    }
  }

  // Sensor noise
  std::default_random_engine generator;
  std::normal_distribution<float> noise(0, noise_variance);

  pcl::PointCloud<pcl::PointXYZ> full_lidar_scan = lidar_scan;
  lidar_scan.clear();
  for (auto i : full_lidar_scan.points) {
    if (sqrt(pow(i.x, 2) + pow(i.y, 2) + pow(i.z, 2)) < range_of_lidar) {
      i.x = i.x + noise(generator);
      i.y = i.y + noise(generator);
      i.z = i.z + noise(generator);

      lidar_scan.push_back(i);
    }
  }
}
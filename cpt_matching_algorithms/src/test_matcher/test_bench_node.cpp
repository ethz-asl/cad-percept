#include <ros/ros.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cgal_msgs/TriangleMeshStamped.h>
#include <chrono>
#include <random>

#include "plane_extraction/plane_extraction.h"
#include "plane_matching/plane_matching.h"
#include "test_matcher/bounded_planes.h"

using namespace cad_percept;
using namespace matching_algorithms;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pcl::PointCloud<pcl::PointXYZ> lidar_scan;
std::vector<float> bin_elevation;
float noise_variance;
float range_of_lidar;
float dtheta;

ros::Publisher scan_pub;
ros::Publisher plane_pub;
ros::Subscriber map_sub;
cad_percept::cgal::MeshModel::Ptr reference_mesh;
bool map_ready = false;
std::string tf_map_frame = "/map";
std::string tf_lidar_frame = "marker_pose";

Eigen::Vector3d gt_translation;
Eigen::Quaterniond gt_rotation;
float transform_TR_[7] = {0, 0, 0, 0, 0, 0, 0};
cad_percept::cgal::Transformation ctransformation;

BoundedPlanes *map_planes;
PointCloud sampled_map;

void samplePose();
void simulateLidar(cad_percept::cgal::Transformation ctransformation,
                   cad_percept::cgal::MeshModel mesh);
void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in);
void runTestIterations();
void goicpMatch(PointCloud lidar_scan_, PointCloud sample_map_, float (&transform_TR_)[7]);

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
  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("extracted_planes", 1, true);
  std::cout << "Tab Enter to start tests" << std::endl;
  std::cin.ignore();

  ros::spin();

  return 0;
}

// Get CAD and sample points
void getCAD(const cgal_msgs::TriangleMeshStamped &cad_mesh_in) {
  if (!map_ready) {
    ros::NodeHandle nh_private("~");

    // std::cout << "Processing CAD mesh" << std::endl;
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

    std::string cache_folder = nh_private.param<std::string>("cache_folder", "fail");

    // load data from file
    map_planes = new BoundedPlanes(*reference_mesh, cache_folder);
    map_planes->dispAllPlanes();

    sampled_map.clear();
    float sample_density_ = 20;
    int n_points = reference_mesh->getArea() * sample_density_;
    cad_percept::cpt_utils::sample_pc_from_mesh(reference_mesh->getMesh(), n_points, 0.0,
                                                &sampled_map);

    // std::cout << "CAD ready" << std::endl;
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
    // std::cout << "Start iteration " << iter << std::endl;

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
      // std::cout << "Load real data" << std::endl;
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

    // std::cout << "Start time measurement" << std::endl;

    t_start = std::chrono::steady_clock::now();

    // // Detect planes
    // // Filtering / Preprocessing Point Cloud
    // if (nh_private.param<bool>("useStructureFilter", false)) {
    //   CloudFilter::filterStaticObject(structure_threshold, lidar_scan, static_structure_cloud);
    // }
    // if (nh_private.param<bool>("useVoxelCentroidFilter", false)) {
    //   CloudFilter::filterVoxelCentroid(search_radius, lidar_scan);
    // }

    // if (lidar_scan.size() == 0) {
    //   std::cout << "Can not find any planes in scan, as scan is empty after filtering" <<
    //   std::endl; break;
    // }
    // // Plane Extraction
    // if (!extractor.compare("pclPlaneExtraction")) {
    //   PlaneExtractor::pclPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
    //                                      tf_lidar_frame, plane_pub);
    // } else if (!extractor.compare("rhtPlaneExtraction")) {
    //   PlaneExtractor::rhtPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
    //                                      tf_lidar_frame, plane_pub,
    //                                      PlaneExtractor::loadRhtConfigFromServer());
    // } else if (!extractor.compare("iterRhtPlaneExtraction")) {
    //   PlaneExtractor::iterRhtPlaneExtraction(extracted_planes, plane_normals, lidar_scan,
    //                                          tf_lidar_frame, plane_pub);

    // } else if (!extractor.compare("cgalRegionGrowing")) {
    //   PlaneExtractor::cgalRegionGrowing(extracted_planes, plane_normals, lidar_scan,
    //   tf_lidar_frame,
    //                                     plane_pub);
    // } else {
    //   std::cout << "Error: Could not find given plane extractor" << std::endl;
    //   break;
    // }

    // // Convert planes to PointNormal PointCloud
    // plane_nr = 0;
    // for (auto plane_normal : plane_normals) {
    //   pcl::computeCentroid(extracted_planes[plane_nr], plane_centroid);
    //   norm_point.x = plane_centroid.x;
    //   norm_point.y = plane_centroid.y;
    //   norm_point.z = plane_centroid.z;

    //   norm_point.normal_x = plane_normal[0];
    //   norm_point.normal_y = plane_normal[1];
    //   norm_point.normal_z = plane_normal[2];

    //   // Correct normals considering normal direction convention for matching
    //   if (norm_point.x * norm_point.normal_x + norm_point.y * norm_point.normal_y +
    //           norm_point.z * norm_point.normal_z <
    //       0) {
    //     norm_point.normal_x = -norm_point.normal_x;
    //     norm_point.normal_y = -norm_point.normal_y;
    //     norm_point.normal_z = -norm_point.normal_z;
    //   }

    //   scan_planes.push_back(norm_point);
    //   plane_nr++;
    // }

    // // Match planes
    // results.clear();
    // transform_error = PlaneMatch::PRRUS(transform_TR_, scan_planes, *map_planes, results);

    goicpMatch(lidar_scan, sampled_map, transform_TR_);

    transform_error = 0;
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
      std::cout << std::endl;
      std::cout << "Could not find a set of orthogonal planes" << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      // std::cout << "Could not find orthogonal planes" << std::endl;
      // std::cout << "ground truth position: x: " << gt_translation[0] << " y: " <<
      // gt_translation[1]
      //           << " z: " << gt_translation[2] << std::endl;

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

      // Transform inliers of the plane
      int i = 0;
      if (extracted_planes.size() != 0) {
        for (auto extracted_plane : extracted_planes) {
          extracted_planes[i].clear();
          pcl::transformPointCloud(extracted_plane, extracted_planes[i], res_transform);
          i++;
        }
        PlaneExtractor::visualizePlane(extracted_planes, scan_pub, tf_map_frame);
      } else {
        pcl::transformPointCloud(lidar_scan, lidar_scan, res_transform);
      }
      ros::spinOnce();

      std::cout << std::endl;
      std::cout << "calculated position: x: " << transform_TR_[0] << " y: " << transform_TR_[1]
                << " z: " << transform_TR_[2] << std::endl;
      std::cout << "ground truth position: x: " << gt_translation[0] << " y: " << gt_translation[1]
                << " z: " << gt_translation[2] << std::endl;
      std::cout << "calculated orientation: qw: " << transform_TR_[3] << " qx: " << transform_TR_[4]
                << " qy: " << transform_TR_[5] << " qz: " << transform_TR_[6] << std::endl;
      std::cout << "ground truth orientation: qw: " << gt_rotation.w() << " qx: " << gt_rotation.x()
                << " qy: " << gt_rotation.y() << " qz: " << gt_rotation.z() << std::endl;

      translation_error =
          (Eigen::Vector3d(transform_TR_[0], transform_TR_[1], transform_TR_[2]) - gt_translation)
              .norm();
      std::cout << "translation error " << translation_error << std::endl;

      rotation_error =
          Eigen::AngleAxis<double>(Eigen::Quaterniond(transform_TR_[3], transform_TR_[4],
                                                      transform_TR_[5], transform_TR_[6])
                                       .toRotationMatrix() *
                                   gt_rotation.inverse().toRotationMatrix())
              .angle();
      duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
      // std::cout << "translation error: " << translation_error << std::endl;
      // std::cout << "rotation error: " << rotation_error << std::endl;
      // std::cout << "time needed: " << duration.count() << " milliseconds" << std::endl;

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
    if (in_map_bit_map_garage[x_coord][y_coord]) {
      valid_position = true;
    }
  }
  gt_translation[0] = x_coord - 23;
  gt_translation[1] = y_coord - 7;
  gt_translation[2] = (double)(std::rand() % 10) * 0.1 + 0.5;

  // Uniform sampling
  // double euler_x = (std::rand() % 30) * M_PI / 15 - M_PI / 2;
  // double euler_y = (std::rand() % 30) * M_PI / 15 - M_PI / 2;
  // double euler_z = (std::rand() % 30) * M_PI / 15 - M_PI / 2;

  double euler_x = 0;
  double euler_y = 0;
  double euler_z = (std::rand() % 30) * M_PI / 15 - M_PI / 2;

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

void goicpMatch(PointCloud lidar_scan_, PointCloud sample_map_, float (&transform_TR_)[7]) {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "             Go-ICP matcher started            " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;
  ros::NodeHandle nh_private_("~");

  std::string downsample_points = nh_private_.param<std::string>("GoICPdownsample", "1000");
  std::string goicp_location = nh_private_.param<std::string>("goicp_folder", "fail");

  PointCloud go_icp_lidar = lidar_scan_;
  PointCloud go_icp_map = sample_map_;

  // Find translation for centralization
  pcl::PointXYZ transl_lidar;
  pcl::computeCentroid(go_icp_lidar, transl_lidar);

  pcl::PointXYZ transl_map;
  pcl::computeCentroid(go_icp_map, transl_map);

  // Centralize point clouds
  Eigen::Matrix4d transform_lidar = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation_lidar(transl_lidar.x, transl_lidar.y, transl_lidar.z);
  transform_lidar.block(0, 3, 3, 1) = translation_lidar;
  pcl::transformPointCloud(go_icp_lidar, go_icp_lidar, transform_lidar);

  Eigen::Matrix4d transform_map = Eigen::Matrix4d::Identity();
  Eigen::Vector3d translation_map(transl_map.x, transl_map.y, transl_map.z);
  transform_map.block(0, 3, 3, 1) = translation_map;
  pcl::transformPointCloud(go_icp_map, go_icp_map, transform_map);

  // Find point, which is furthest away from centroid for scaling
  float max_dist_lidar = 0;
  for (auto point : go_icp_lidar.points) {
    if (sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)) >= max_dist_lidar) {
      max_dist_lidar = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    }
  }

  float max_dist_map = 0;
  for (auto point : go_icp_map.points) {
    if (sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2)) >= max_dist_map) {
      max_dist_lidar = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
    }
  }
  float max_dist = std::max(max_dist_map, max_dist_lidar);

  // Scale point cloud to [-1,1]³
  Eigen::Matrix4d trans_scale_lidar = Eigen::Matrix4d::Identity();
  trans_scale_lidar(0, 0) = trans_scale_lidar(0, 0) / max_dist;
  trans_scale_lidar(1, 1) = trans_scale_lidar(1, 1) / max_dist;
  trans_scale_lidar(2, 2) = trans_scale_lidar(2, 2) / max_dist;
  pcl::transformPointCloud(go_icp_lidar, go_icp_lidar, trans_scale_lidar);

  Eigen::Matrix4d trans_scale_map = Eigen::Matrix4d::Identity();
  trans_scale_map(0, 0) = trans_scale_map(0, 0) / max_dist;
  trans_scale_map(1, 1) = trans_scale_map(1, 1) / max_dist;
  trans_scale_map(2, 2) = trans_scale_map(2, 2) / max_dist;
  pcl::transformPointCloud(go_icp_map, go_icp_map, trans_scale_map);

  // Create txt files of point clouds, required for Go-ICP
  chdir(goicp_location.c_str());
  std::ofstream map_file("map.txt");
  map_file << go_icp_map.width << std::endl;
  for (PointCloud::iterator i = go_icp_map.points.begin(); i < go_icp_map.points.end(); i++) {
    map_file << i->x << " " << i->y << " " << i->z << std::endl;
  }
  std::cout << "Map.txt created" << std::endl;
  map_file.close();

  std::ofstream lidar_file("lidar_scan.txt");
  lidar_file << go_icp_lidar.width << std::endl;
  for (PointCloud::iterator i = go_icp_lidar.points.begin(); i < go_icp_lidar.points.end(); i++) {
    lidar_file << i->x << " " << i->y << " " << i->z << std::endl;
  }
  std::cout << "lidar_scan.txt created" << std::endl;
  lidar_file.close();

  std::cout << "Start Go-ICP" << std::endl;
  std::string command =
      "./GoICP map.txt lidar_scan.txt " + downsample_points + " config.txt output.txt";
  system(command.c_str());
  std::cout << "Go-ICP finished" << std::endl;

  // Read results
  float time_needed;
  Eigen::Matrix4d go_icp_trans = Eigen::Matrix4d::Identity();

  std::ifstream output("output.txt");
  if (output.is_open()) {
    std::vector<float> values;
    std::copy(std::istream_iterator<float>(output), std::istream_iterator<float>(),
              std::back_inserter(values));

    time_needed = values[0];
    go_icp_trans(0, 0) = values[1];
    go_icp_trans(0, 1) = values[2];
    go_icp_trans(0, 2) = values[3];
    go_icp_trans(1, 0) = values[4];
    go_icp_trans(1, 1) = values[5];
    go_icp_trans(1, 2) = values[6];
    go_icp_trans(2, 0) = values[7];
    go_icp_trans(2, 1) = values[8];
    go_icp_trans(2, 2) = values[9];
    go_icp_trans(0, 3) = values[10];
    go_icp_trans(1, 3) = values[11];
    go_icp_trans(2, 3) = values[12];

    std::cout << "Go-ICP needed " << time_needed << " seconds." << std::endl;
  } else {
    std::cout << "Could not read output file" << std::endl;
  }

  // Get matrix of unscaled matrix
  Eigen::Matrix4d final_transf = go_icp_trans;

  Eigen::Matrix3d final_rot = final_transf.block(0, 0, 3, 3);
  Eigen::Quaterniond final_q(final_rot);

  // Revert scaling and translation
  transform_TR_[0] = final_transf(0, 3) * max_dist - transl_lidar.x + transl_map.x;
  transform_TR_[1] = final_transf(1, 3) * max_dist - transl_lidar.y + transl_map.y;
  transform_TR_[2] = final_transf(2, 3) * max_dist - transl_lidar.z + transl_map.z;

  transform_TR_[3] = final_q.w();
  transform_TR_[4] = final_q.x();
  transform_TR_[5] = final_q.y();
  transform_TR_[6] = final_q.z();
}
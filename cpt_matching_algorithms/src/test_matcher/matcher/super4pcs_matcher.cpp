#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/super4pcs.h>
#include <super4pcs/algorithms/super4pcs.h>
#include "super4pcs_demo_utils.h"
#include "test_matcher/test_matcher.h"
namespace cad_percept {
namespace matching_algorithms {

void TestMatcher::super4pcs_match() {
  std::cout << "///////////////////////////////////////////////" << std::endl;
  std::cout << "            Super4PCS matcher started          " << std::endl;
  std::cout << "///////////////////////////////////////////////" << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr super4pcs_lidar(new pcl::PointCloud<pcl::PointXYZ>);
  *super4pcs_lidar = lidar_frame;
  pcl::PointCloud<pcl::PointXYZ>::Ptr super4pcs_map(new pcl::PointCloud<pcl::PointXYZ>);
  *super4pcs_map = sample_map;
  pcl::PointCloud<pcl::PointNormal>::Ptr super4pcs_lidar_aligned(
      new pcl::PointCloud<pcl::PointNormal>);

  // Compute normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normest_lidar;
  normest_lidar.setInputCloud(super4pcs_lidar);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_lidar(new pcl::search::KdTree<pcl::PointXYZ>());
  normest_lidar.setSearchMethod(tree_lidar);
  pcl::PointCloud<pcl::PointNormal>::Ptr normal_lidar(new pcl::PointCloud<pcl::PointNormal>);
  normest_lidar.setRadiusSearch(0.1);
  normest_lidar.compute(*normal_lidar);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normest_map;
  normest_map.setInputCloud(super4pcs_map);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_map(new pcl::search::KdTree<pcl::PointXYZ>());
  normest_map.setSearchMethod(tree_map);
  pcl::PointCloud<pcl::PointNormal>::Ptr normal_map(new pcl::PointCloud<pcl::PointNormal>);
  normest_map.setRadiusSearch(0.1);
  normest_map.compute(*normal_map);

  // Load Super4PCS parameters
  std::vector<std::string> arguments;
  arguments.push_back("program name");
  arguments.push_back("scene.obj");  // just a dummy
  arguments.push_back("model.obj");  // just a dummy
  arguments.push_back("-n 1000 -d 0.1");
  std::vector<char*> argv_new;
  argv_new.reserve(arguments.size());
  for (int i = 0; i < arguments.size(); i++) {
    argv_new.push_back(const_cast<char*>(arguments[i].c_str()));
  }

  GlobalRegistration::Demo::getArgs(4, argv_new.data());
  pcl::Super4PCS<pcl::PointNormal, pcl::PointNormal> align;
  GlobalRegistration::Demo::setOptionsFromArgs(align.options_);

  // Perform alignment
  align.setInputSource(normal_lidar);
  align.setInputTarget(normal_map);

  {
    pcl::ScopeTime t("Alignment");
    align.align(*super4pcs_lidar_aligned);
  }

  Eigen::Matrix4f final_transf = align.getFinalTransformation().cast<float>();
  Eigen::Matrix3f final_rot = final_transf.block(0, 0, 3, 3);
  Eigen::Vector3f final_euler = final_rot.eulerAngles(0, 1, 2);

  // Revert scaling and translation
  transform_TR[0] = final_transf(0, 3);
  transform_TR[1] = final_transf(1, 3);
  transform_TR[2] = final_transf(2, 3);

  transform_TR[3] = final_euler(0);
  transform_TR[4] = final_euler(1);
  transform_TR[5] = final_euler(2);
}

}  // namespace matching_algorithms
}  // namespace cad_percept

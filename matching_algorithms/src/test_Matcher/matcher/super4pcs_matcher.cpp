#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/super4pcs.h>
#include <super4pcs/shared4pcs.h>
#include "../../../Super4PCS/demos/demo-utils.h"
#include "test_Matcher/test_Matcher.h"
namespace cad_percept {
namespace matching_algorithms {
using namespace GlobalRegistration;
// Not working yet
void test_Matcher::super4pcs_match(float (&transformTR)[6]) {
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
  arguments.push_back("scene.obj");
  arguments.push_back("model.obj");
  arguments.push_back("-o 0.5 -n 1000");
  std::vector<char*> argv_new;
  argv_new.reserve(arguments.size());

  for (int i = 0; i < arguments.size(); i++) {
    argv_new.push_back(const_cast<char*>(arguments[i].c_str()));
  }

  Demo::getArgs(4, argv_new.data());
  pcl::Super4PCS<pcl::PointNormal, pcl::PointNormal> align;
  Demo::setOptionsFromArgs(align.options_);
  {
    pcl::ScopeTime t("Alignment");
    align.align(*super4pcs_lidar_aligned);
  }

  // Perform alignment
  align.setInputSource(normal_lidar);
  align.setInputTarget(normal_map);
  Eigen::Matrix4f final_transf = align.getFinalTransformation();
  Eigen::Matrix3d final_rot = final_transf.block(0, 0, 3, 3);
  Eigen::Vector3f final_euler = final_rot.cast<float>().eulerAngles(0, 1, 2);

  // Revert scaling and translation
  transformTR[0] = final_transf(0, 3);
  transformTR[1] = final_transf(1, 3);
  transformTR[2] = final_transf(2, 3);

  transformTR[3] = final_euler(0);
  transformTR[4] = final_euler(1);
  transformTR[5] = final_euler(2);
}

}  // namespace matching_algorithms
}  // namespace cad_percept

#include "plane_matching/plane_matching_lib.h"

void PlaneMatchLib::prrus(float (&transfromTR)[7],
                          const pcl::PointCloud<pcl::PointNormal> scan_planes,
                          const pcl::PointCloud<pcl::PointNormal> map_planes) {
  std::cout << "////  PRRUS Matching Started  ////" << std::endl;
  std::cout << "Number of planes in map: " << map_planes.size() << std::endl;
  std::cout << "Number of planes in scan: " << scan_planes.size() << std::endl;

  float match_score = -1;
  std::vector<int> plane_assignment(scan_planes.size(), 0);

  // ToDo Matching part

  for (int plane_nr; plane_nr < scan_planes.size(); plane_nr++) {
    std::cout << "Plane " << plane_nr << " from Scan matched with " << plane_assignment[plane_nr]
              << " from mesh" << std::endl;
  }
  transform_split(transfromTR, plane_assignment, scan_planes, map_planes);
}

void PlaneMatchLib::transform_split(float (&transfromTR)[7], std::vector<int> plane_assignement,
                                    const pcl::PointCloud<pcl::PointNormal> scan_planes,
                                    const pcl::PointCloud<pcl::PointNormal> map_planes) {
  std::cout << "Calculate Transformation via plane correspondences" << std::endl;
};

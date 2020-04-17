#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <cgal_definitions/cgal_typedefs.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_utils/pc_processing.h>

#include <plane_extraction/plane_extraction.h>

using namespace cad_percept::cgal;
using namespace cad_percept::matching_algorithms;

// A modifier creating a triangle with the incremental builder.
template <class HDS>
class TestingMesh : public CGAL::Modifier_base<HDS> {
 public:
  TestingMesh() {}
  void operator()(HDS& hds) {
    // Create a simple plane parallel to y - z - plane at x = 10
    CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
    B.begin_surface(3, 3);
    B.add_vertex(Point(10, 0, 0));
    B.add_vertex(Point(10, 10, 0));
    B.add_vertex(Point(10, 0, 10));

    B.begin_facet();
    B.add_vertex_to_facet(0);
    B.add_vertex_to_facet(1);
    B.add_vertex_to_facet(2);
    B.end_facet();

    B.end_surface();
  }
};

TEST(PlaneExtractionTest, pclPlaneExtraction) {
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);

  MeshModel::Ptr model;
  MeshModel::create(m, &model);

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  cad_percept::cpt_utils::sample_pc_from_mesh(model->getMesh(), 100, 0.0, &pointcloud);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_planes;
  std::vector<Eigen::Vector3d> plane_coefficients;
  std::string tf_map_frame = "map";
  ros::Publisher plane_pub_;

  PlaneExtractor::pclPlaneExtraction(extracted_planes, plane_coefficients, pointcloud, tf_map_frame,
                                     plane_pub_);
  std::cout << "pclPlaneExtraction found plane at rho: " << plane_coefficients[0][0]
            << " theta: " << plane_coefficients[0][1] << " psi: " << plane_coefficients[0][2]
            << std::endl;

  EXPECT_TRUE(std::abs(plane_coefficients[0][0] - 10) < 0.3 &&
              std::abs(plane_coefficients[0][1] - 0) < 0.3 &&
              std::abs(plane_coefficients[0][2] - 0) < 0.3);
}

TEST(PlaneExtractionTest, rhtPlaneExtraction) {
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);

  MeshModel::Ptr model;
  MeshModel::create(m, &model);

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  cad_percept::cpt_utils::sample_pc_from_mesh(model->getMesh(), 100, 0.0, &pointcloud);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_planes;
  std::vector<Eigen::Vector3d> plane_coefficients;
  std::string tf_map_frame = "map";
  ros::Publisher plane_pub_;

  PlaneExtractor::rhtPlaneExtraction(extracted_planes, plane_coefficients, pointcloud, tf_map_frame,
                                     plane_pub_);

  std::cout << "rhtPlaneExtraction found plane at rho: " << plane_coefficients[0][0]
            << " theta: " << plane_coefficients[0][1] << " psi: " << plane_coefficients[0][2]
            << std::endl;

  EXPECT_TRUE(std::abs(plane_coefficients[0][0] - 10) < 0.3 &&
              std::abs(plane_coefficients[0][1] - 0) < 0.3 &&
              std::abs(plane_coefficients[0][2] - 0) < 0.3);
}

TEST(PlaneExtractionTest, interRhtPlaneExtraction) {
  Polyhedron m;
  TestingMesh<HalfedgeDS> testcase;
  m.delegate(testcase);

  MeshModel::Ptr model;
  MeshModel::create(m, &model);

  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  cad_percept::cpt_utils::sample_pc_from_mesh(model->getMesh(), 100, 0.0, &pointcloud);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> extracted_planes;
  std::vector<Eigen::Vector3d> plane_coefficients;
  std::string tf_map_frame = "map";
  ros::Publisher plane_pub_;

  PlaneExtractor::iterRhtPlaneExtraction(extracted_planes, plane_coefficients, pointcloud,
                                         tf_map_frame, plane_pub_);

  std::cout << "iterRhtPlaneExtraction found plane at rho: " << plane_coefficients[0][0]
            << " theta: " << plane_coefficients[0][1] << " psi: " << plane_coefficients[0][2]
            << std::endl;

  EXPECT_TRUE(std::abs(plane_coefficients[0][0] - 10) < 0.3 &&
              std::abs(plane_coefficients[0][1] - 0) < 0.3 &&
              std::abs(plane_coefficients[0][2] - 0) < 0.3);
}
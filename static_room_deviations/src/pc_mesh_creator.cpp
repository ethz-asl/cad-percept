#include "static_room_deviations/pc_mesh_creator.h"

namespace cad_percept {
namespace cgal {

// A modifier creating a triangle with the incremental builder.
template <class HDS>
class SamplePolyhedron : public CGAL::Modifier_base<HDS> {
 public:
  SamplePolyhedron() {}
  void operator()(HDS& hds) {
    // Postcondition: hds is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HDS> P(hds, true);

    Point p0(0, 0, 0);
    Point p1(4, 0, 0);
    Point p2(4, 1.5, 0);
    Point p3(5, 1.5, 0);
    Point p4(5, 3, 0);
    Point p5(0, 3, 0);
    Point p6(0, 0, 2);
    Point p7(4, 0, 2);
    Point p8(4, 1.5, 2);
    Point p9(5, 1.5, 2);
    Point p10(5, 3, 2);
    Point p11(0, 3, 2);

    P.begin_surface(12, 20);

    P.add_vertex(p0);
    P.add_vertex(p1);
    P.add_vertex(p2);
    P.add_vertex(p3);
    P.add_vertex(p4);
    P.add_vertex(p5);
    P.add_vertex(p6);
    P.add_vertex(p7);
    P.add_vertex(p8);
    P.add_vertex(p9);
    P.add_vertex(p10);
    P.add_vertex(p11);

    P.begin_facet();
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(7);
    P.add_vertex_to_facet(6);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(1);
    P.add_vertex_to_facet(7);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(1);
    P.add_vertex_to_facet(8);
    P.add_vertex_to_facet(7);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(1);
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(8);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(9);
    P.add_vertex_to_facet(8);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(3);
    P.add_vertex_to_facet(9);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(3);
    P.add_vertex_to_facet(10);
    P.add_vertex_to_facet(9);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(3);
    P.add_vertex_to_facet(4);
    P.add_vertex_to_facet(10);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(4);
    P.add_vertex_to_facet(11);
    P.add_vertex_to_facet(10);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(4);
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(11);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(6);
    P.add_vertex_to_facet(11);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(6);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(1);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(2);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(4);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(4);
    P.add_vertex_to_facet(3);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(6);
    P.add_vertex_to_facet(7);
    P.add_vertex_to_facet(8);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(6);
    P.add_vertex_to_facet(8);
    P.add_vertex_to_facet(11);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(11);
    P.add_vertex_to_facet(8);
    P.add_vertex_to_facet(10);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(10);
    P.add_vertex_to_facet(8);
    P.add_vertex_to_facet(9);
    P.end_facet();

    P.end_surface();
  }
};

// A modifier creating a triangle with the incremental builder.
template <class HDS>
class SamplePolyhedronDeviated : public CGAL::Modifier_base<HDS> {
 public:
  SamplePolyhedronDeviated() {}
  void operator()(HDS& hds) {
    // Postcondition: hds is a valid polyhedral surface.
    CGAL::Polyhedron_incremental_builder_3<HDS> P(hds, true);

    Point p0(0, 0, 0);
    Point p1(4, 0, 0);
    Point p2(4, 1.5, 0);
    Point p3(5, 1.5, 0);
    Point p4(6, 4, 0);
    Point p5(0, 3, 0);
    Point p6(0, 0, 2);
    Point p7(4, 0, 2);
    Point p8(4, 1.5, 2);
    Point p9(5, 1.5, 2);
    Point p10(6, 4, 2);    
    Point p11(0, 3, 2);

    P.begin_surface(12, 20);

    P.add_vertex(p0);
    P.add_vertex(p1);
    P.add_vertex(p2);
    P.add_vertex(p3);
    P.add_vertex(p4);
    P.add_vertex(p5);
    P.add_vertex(p6);
    P.add_vertex(p7);
    P.add_vertex(p8);
    P.add_vertex(p9);
    P.add_vertex(p10);
    P.add_vertex(p11);

    P.begin_facet();
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(7);
    P.add_vertex_to_facet(6);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(1);
    P.add_vertex_to_facet(7);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(1);
    P.add_vertex_to_facet(8);
    P.add_vertex_to_facet(7);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(1);
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(8);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(9);
    P.add_vertex_to_facet(8);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(3);
    P.add_vertex_to_facet(9);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(3);
    P.add_vertex_to_facet(10);
    P.add_vertex_to_facet(9);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(3);
    P.add_vertex_to_facet(4);
    P.add_vertex_to_facet(10);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(4);
    P.add_vertex_to_facet(11);
    P.add_vertex_to_facet(10);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(4);
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(11);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(6);
    P.add_vertex_to_facet(11);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(6);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(1);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(0);
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(2);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(5);
    P.add_vertex_to_facet(4);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(2);
    P.add_vertex_to_facet(4);
    P.add_vertex_to_facet(3);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(6);
    P.add_vertex_to_facet(7);
    P.add_vertex_to_facet(8);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(6);
    P.add_vertex_to_facet(8);
    P.add_vertex_to_facet(11);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(11);
    P.add_vertex_to_facet(8);
    P.add_vertex_to_facet(10);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(10);
    P.add_vertex_to_facet(8);
    P.add_vertex_to_facet(9);
    P.end_facet();

    P.end_surface();
  }
};

void build_sample_polyhedrons(Polyhedron *P, Polyhedron *P_deviated) {
  SamplePolyhedron<HalfedgeDS> samplepoly;
  SamplePolyhedronDeviated<HalfedgeDS> samplepolydev;
  P->delegate(samplepoly);
  P_deviated->delegate(samplepolydev);
  if (P->is_valid()) {
    std::cout << "P is valid" << std::endl;
  }
  if (P_deviated->is_valid()) {
    std::cout << "P_deviated is valid" << std::endl;
  }

  std::ofstream off_file1("/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/room.off", std::ios::binary);
  CGAL::write_off(off_file1, *P);

  std::ofstream off_file2("/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/room_deviated.off", std::ios::binary);
  CGAL::write_off(off_file2, *P_deviated);
}

void sample_pc_from_mesh(const Polyhedron &P, 
                         const int no_of_points,
                         const double stddev,
                         PointCloud *pointcloud,
                         std::string file_name) {
  // generate random point sets on triangle mesh
  std::vector<Point> points;
  // Create the generator, input is the Polyhedron P
  CGAL::Random_points_in_triangle_mesh_3<Polyhedron> g(P);
  // Get no_of_points random points in cdt
  CGAL::cpp11::copy_n(g, no_of_points, std::back_inserter(points));
  // Check that we have really created no_of_points points.
  assert(points.size() == no_of_points);
  // print the first point that was generated
  //std::cout << points[0] << std::endl;  

  // add random noise with Gaussian distribution
  const double mean = 0.0;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);

  std::vector<Point> points_noise;
  for (auto point : points) {
    Point p_noise;
    p_noise = Point(point.x() + dist(generator),
                    point.y() + dist(generator),
                    point.z() + dist(generator));
    points_noise.push_back(p_noise);
  }

  // save as pcd
  pointcloud->width = points_noise.size();
  pointcloud->height = 1;
  pointcloud->header.frame_id = "mesh";
  pointcloud->is_dense = false;

  for (auto point : points_noise) {
    pcl::PointXYZ cloudpoint;
    cloudpoint.x = (float)point.x();
    cloudpoint.y = (float)point.y();
    cloudpoint.z = (float)point.z();
    pointcloud->push_back(cloudpoint);
  }

  std::stringstream ss;
  ss << "/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/" << file_name << ".pcd";
  pcl::io::savePCDFileASCII(ss.str(), *pointcloud);
  std::cerr << "Saved " << pointcloud->points.size() << " data points to pcd" << std::endl;
}

}
}
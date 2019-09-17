/**
 * Creating a sample Polyhedron and PointCloud for testing purposes.
 */
#include "relative_deviations/pc_mesh_creator.h"

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

    // adding a wrong wall:
    Point p12(4, 0.75, 0);
    Point p13(4, 0.75, 2);

    P.begin_surface(16, 22);

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
    P.add_vertex(p12);
    P.add_vertex(p13);
    P.add_vertex(p0);
    P.add_vertex(p6);

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

    P.begin_facet();
    P.add_vertex_to_facet(14);
    P.add_vertex_to_facet(13);
    P.add_vertex_to_facet(15);
    P.end_facet();

    P.begin_facet();
    P.add_vertex_to_facet(14);
    P.add_vertex_to_facet(12);
    P.add_vertex_to_facet(13);
    P.end_facet();

    P.end_surface();
  }
};

void build_sample_polyhedrons(Polyhedron* P, Polyhedron* P_deviated) {
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

  std::ofstream off_file1(
      "/home/julian/megabot_ws/src/cad-percept/srd_relative_deviations/resources/room.off",
      std::ios::binary);
  CGAL::write_off(off_file1, *P);

  std::ofstream off_file2(
      "/home/julian/megabot_ws/src/cad-percept/srd_relative_deviations/resources/room_deviated.off",
      std::ios::binary);
  CGAL::write_off(off_file2, *P_deviated);
}

}  // namespace cgal
}  // namespace cad_percept

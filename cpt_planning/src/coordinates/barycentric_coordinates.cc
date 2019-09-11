#include <cpt_planning/coordinates/barycentric_coordinates.h>

namespace cad_percept {
namespace planning {

BarycentricCoordinates::BarycentricCoordinates(cgal::Vector3In& values) { coordinates_ = values; }

cgal::Vector3Return BarycentricCoordinates::asVector() { return coordinates_; }

cgal::Vector3Return BarycentricCoordinates::toCartesian(TupleTriangle& triangle) {
  return toCartesian(std::get<0>(triangle), std::get<1>(triangle), std::get<2>(triangle));
}

void BarycentricCoordinates::fromCartesian(TupleTriangle& triangle,
                                           cgal::Vector3In& point_on_triangle) {
  fromCartesian(std::get<0>(triangle), std::get<1>(triangle), std::get<2>(triangle),
                point_on_triangle);
}

}  // namespace planning
}  // namespace cad_percept

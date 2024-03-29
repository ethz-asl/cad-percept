#include "cpt_utils/cpt_utils.h"

#include <random>

namespace cad_percept {
namespace cpt_utils {

void intersection(const cgal::Plane &plane, const cgal::Line &line, cgal::Point *point) {
  CGAL::cpp11::result_of<cgal::Intersect(cgal::Line, cgal::Plane)>::type result =
      CGAL::intersection(line, plane);
  if (result) {
    if (const cgal::Point *p = boost::get<cgal::Point>(&*result)) {
      *point = *p;
      // std::cout << "Intersection Point: " << *p << std::endl;
    } else {
      const cgal::Line *l = boost::get<cgal::Line>(&*result);
      // std::cout << "Intersection is a line: " << *l << std::endl;
    }
  }
}

cgal::Point closestPointOnPlane(const cgal::Plane &plane, const cgal::Point &point) {
  // Raycast into direction of triangle normal
  cgal::Vector normal = getNormalFromPlane(plane);
  cgal::Line line(point, normal);
  cgal::Point intersec_point;
  intersection(plane, line, &intersec_point);
  return intersec_point;
}

cgal::Vector getNormalFromPlane(const cgal::Plane &plane) {
  cgal::Vector normal(plane.a(), plane.b(), plane.c());
  normal = normal / sqrt(normal.squared_length());
  return normal;
}

Associations associatePointCloud(const PointCloud &pc_msg, cgal::MeshModel::Ptr mesh_model) {
  // Convert point cloud msg
  std::cout << "Associating pointcloud of size " << pc_msg.width << " x " << pc_msg.height
            << std::endl;
  Associations associations;
  associations.points_from.resize(3, pc_msg.width);  // 3 rows, width columns
  associations.points_to.resize(3, pc_msg.width);
  associations.distances.resize(pc_msg.width);
  associations.triangles_to.resize(pc_msg.width);
  for (size_t i = 0u; i < pc_msg.width; ++i) {
    // loop through all points of point cloud

    cgal::PointAndPrimitiveId ppid =
        mesh_model->getClosestTriangle(pc_msg[i].x, pc_msg[i].y, pc_msg[i].z);
    cgal::Point pt = ppid.first;

    std::string triangle_id = mesh_model->getIdFromFacetHandle(ppid.second);

    associations.points_from(0, i) = pc_msg[i].x;
    associations.points_from(1, i) = pc_msg[i].y;
    associations.points_from(2, i) = pc_msg[i].z;

    // Raycast into direction of triangle normal.
    Eigen::Vector3d normal = cgal::cgalVectorToEigenVector(mesh_model->getNormal(ppid));
    normal.normalize();
    Eigen::Vector3d relative = Eigen::Vector3d(pt.x(), pt.y(), pt.z()) -
                               Eigen::Vector3d(pc_msg[i].x, pc_msg[i].y, pc_msg[i].z);
    Eigen::Vector3d direction = normal.dot(relative) * normal;  // but relative is already in
                                                                // direction of normal because of
                                                                // getClosestTriangle (?!)

    associations.points_to(0, i) =
        associations.points_from(0, i) + direction(0);  // points_to should be pt, right?
    associations.points_to(1, i) = associations.points_from(1, i) + direction(1);
    associations.points_to(2, i) = associations.points_from(2, i) + direction(2);
    associations.distances(i) = direction.norm();
    associations.triangles_to[i] = triangle_id;
  }
  return associations;
}

cgal::Point centerOfBbox(const CGAL::Bbox_3 &bbox) {
  double x = bbox.xmin() + (bbox.xmax() - bbox.xmin()) / 2;
  double y = bbox.ymin() + (bbox.ymax() - bbox.ymin()) / 2;
  double z = bbox.zmin() + (bbox.zmax() - bbox.zmin()) / 2;
  return cgal::Point(x, y, z);
}

cgal::Point centerOfBbox(const PointCloud &pointcloud) {
  CGAL::Bbox_3 bbox;
  computePCBbox(pointcloud, &bbox);
  return centerOfBbox(bbox);
}

void bboxDiameters(const CGAL::Bbox_3 bbox, double *width, double *height) {
  *width = sqrt(pow(bbox.xmax() - bbox.xmin(), 2) + pow(bbox.ymax() - bbox.ymin(), 2));
  *height = sqrt(bbox.zmax() - bbox.zmin());
}

void samplePointsFromMesh(const cgal::Polyhedron &P, const int no_of_points, const double stddev,
                          std::vector<cgal::Point> *points) {
  CHECK(points);

  // generate random point sets on triangle mesh
  std::vector<cgal::Point> points_mesh;
  // Create the generator, input is the Polyhedron P
  CGAL::Random_points_in_triangle_mesh_3<cgal::Polyhedron> g(P);
  // Get no_of_points random points in cdt
  CGAL::cpp11::copy_n(g, no_of_points, std::back_inserter(points_mesh));
  // Check that we have really created no_of_points points.
  assert(points_mesh.size() == no_of_points);
  // print the first point that was generated
  // std::cout << points[0] << std::endl;

  // add random noise with Gaussian distribution
  const double mean = 0.0;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);

  points->reserve(no_of_points);
  for (auto point : points_mesh) {
    cgal::Point p_noise;
    p_noise = cgal::Point(point.x() + dist(generator), point.y() + dist(generator),
                          point.z() + dist(generator));
    points->push_back(p_noise);
  }
}

}  // namespace cpt_utils
}  // namespace cad_percept

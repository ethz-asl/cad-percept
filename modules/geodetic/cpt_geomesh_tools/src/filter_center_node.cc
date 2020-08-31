#include <CGAL/IO/Polyhedron_VRML_1_ostream.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polygon_mesh_processing/clip.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/internal/repair_extra.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <cgal_definitions/mesh_model.h>
#include <cpt_geomesh_tools/filter_center_node.h>
#include <geotf/geodetic_converter.h>

#include <fstream>
#include <iostream>

namespace cad_percept::geomesh_tools {

void FilterCenterNode::setMap(const std::string &path, const std::string &frame) {
  if (frame != name_3857_) {
    std::cout << "Input map has to be in frame EPSG:3857" << std::endl;
    return;
  }

  cad_percept::cgal::MeshModel::create(path, &model_);
}

void FilterCenterNode::setROI(std::vector<Eigen::Vector3d> roi, const std::string &frame) {
  if (!geotf_.canConvert(frame, name_3857_)) {
    std::cout << "Cannot convert frame " << frame << std::endl;
    return;
  }

  roi_points_3857_.resize(roi.size());
  for (int i = 0; i < roi.size(); i++) {
    geotf_.convert(frame, roi[i], name_3857_, &roi_points_3857_[i]);
  }
}

void FilterCenterNode::clipMesh() {
  auto mesh = model_->getMeshNonConst();

  // Fix self intersections (magic)
  if (CGAL::Polygon_mesh_processing::does_self_intersect(*mesh)) {
    CGAL::Polygon_mesh_processing::remove_self_intersections(*mesh);
    std::cout << "Self intersection detected and fixed" << std::endl;
  }

  // for now, we only care about x-y clipping
  for (int i = 0; i < roi_points_3857_.size(); i++) {
    int next_index = (i + 1) % roi_points_3857_.size();
    cgal::Point p0(roi_points_3857_[i].x(), roi_points_3857_[i].y(), 0);
    cgal::Point p1(roi_points_3857_[next_index].x(), roi_points_3857_[next_index].y(), 0);
    cgal::Vector offset(0, 0, 100);

    CGAL::Polygon_mesh_processing::clip(*mesh, cgal::Plane(p0, p0 + offset, p1));
  }
}

void FilterCenterNode::generateMesh(const std::string &output_path,
                                    const std::string &output_frame) {
  model_->transform([&, this](const cgal::Point &n) {
    Eigen::Vector3d output;
    this->geotf_.convert(this->name_3857_, {n.x(), n.y(), n.z()}, output_frame, &output);
    return cgal::Point(output.x(), output.y(), output.z());
  });

  std::ofstream file;
  file.open(output_path, std::ios::binary);
  file << std::fixed;
  file << std::setprecision(15);
  file << model_->getMesh();
  file.flush();
  file.close();
}

}  // namespace cad_percept::geomesh_tools

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "filtercenternode");
  ros::NodeHandle nh("~");

  // Param loading ( no error checking whatsoever at the moment)
  std::string input_path, output_path, roi_frame, output_frame;
  std::vector<double> roi_x, roi_y;

  nh.getParam("input_path", input_path);
  nh.getParam("output_path", output_path);
  nh.getParam("roi_frame", roi_frame);
  nh.getParam("output_frame", output_frame);

  nh.getParam("roi_x", roi_x);
  nh.getParam("roi_y", roi_y);
  assert(roi_x.size() == roi_y.size());
  std::vector<Eigen::Vector3d> points;
  points.resize(roi_x.size());
  for (size_t i = 0; i < roi_x.size(); i++) {
    points[i] = {roi_x[i], roi_y[i], 0.0};  // we don't care about z at the moment.
    std::cout << points[i] << std::endl;
  }

  // set up mesh and process
  cad_percept::geomesh_tools::FilterCenterNode node;
  node.init();
  node.setMap(input_path, "Pseudo-Mercator");
  node.setROI(points, roi_frame);
  node.clipMesh();
  node.generateMesh(output_path, output_frame);
  return 0;

}



#include "static_room_deviations/static_room_deviations_ros.h"

namespace cad_percept {
namespace room_deviations {

StaticRoomDeviations::StaticRoomDeviations(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    map_frame_(nh_private.param<std::string>("map_frame", "fail")),
    cb(10) { // use 10 latest scans for detection

  ref_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("ref_mesh", 100, true); // latching to true
  reading_pc_pub_ = nh_.advertise<PointCloud>("reading_pc_pub", 1, true);
  icp_pc_pub_ = nh_.advertise<PointCloud>("icp_pc_pub", 1, true);
  reconstructed_planes_pub_ = nh_.advertise<ColoredPointCloud>("reconstructed_planes_pub", 1, true);
  polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("polygon_pub", 1, true);

  // manually starting test case
  if (nh_private_.param<bool>("test", "fail") == 1) {
    cgal::PointCloud reading_pc;
    createTestCase(&reading_pc);
    deviations.init(nh_private_.param<std::string>("reference_model_file", "fail").c_str());
    readingCallback(reading_pc);
  }
  else {
    deviations.init(nh_private_.param<std::string>("reference_model_file", "fail").c_str());
  }
}

StaticRoomDeviations::~StaticRoomDeviations() {}

void StaticRoomDeviations::createTestCase(cgal::PointCloud *reading_pc) {
  cgal::Polyhedron P;
  cgal::Polyhedron P_deviated;
  cgal::build_sample_polyhedrons(&P, &P_deviated);
  cgal::sample_pc_from_mesh(P_deviated, 3000, 0.01, reading_pc, "reading_pc"); 

  // transform reading pointcloud a little bit to test ICP
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 0.5, 0.1, 0.2;
  float theta = M_PI*0.01;
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  transformPointCloud(reading_pc, transform);
  pcl::io::savePCDFileASCII("/home/julian/cadify_ws/src/mt_utils/static_room_deviations/resources/deviated_reading_pc.pcd", *reading_pc);
}

void StaticRoomDeviations::readingCallback(cgal::PointCloud &reading_pc) {
  publishMesh(deviations.reference_mesh, &ref_mesh_pub_);
  std::vector<reconstructed_plane> rec_planes;
  PointCloud icp_cloud;
  std::ifstream ifs_icp_config(nh_private_.param<std::string>("icp_configuration_file", "fail").c_str());
  std::ifstream ifs_selective_icp_config(nh_private_.param<std::string>("selective_icp_configuration_file", "fail").c_str());
  std::ifstream ifs_normal_filter(nh_private_.param<std::string>("normal_filter_file", "fail").c_str());
  deviations.detectChanges(&rec_planes, reading_pc, &icp_cloud, ifs_icp_config, ifs_normal_filter, ifs_selective_icp_config);
  publishReconstructedPlanes(rec_planes, &reconstructed_planes_pub_); 
  //cgal::Polyhedron P = deviations.reference_mesh_merged.getMesh();
  //publishPolyhedron(P);
  publishCloud<PointCloud>(&reading_pc, &reading_pc_pub_);
  publishCloud<PointCloud>(&icp_cloud, &icp_pc_pub_);
}

void StaticRoomDeviations::bufferCallback(cgal::PointCloud &reading_pc) {
  // insert reading_pc in buffer
  // the reading_pc should be pre-aligned with model here before continueing
  cb.push_back(reading_pc);
  if (cb.full()) {
    cgal::PointCloud aligned_pc;
    align_sequence(cb, &aligned_pc);
    readingCallback(aligned_pc);
  }
}

void StaticRoomDeviations::publishMesh(const cgal::MeshModel &model, ros::Publisher *publisher) const {
  cgal::Polyhedron P;
  P = model.getMesh();
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::triangleMeshToMsg(P, &t_msg);
  c_msg.mesh = t_msg;

  std_msgs::ColorRGBA c;
  c.r = 0.0;
  c.g = 0.0;
  c.b = 1.0;
  c.a = 0.8;
  c_msg.color = c;

  c_msg.header.frame_id = map_frame_;
  c_msg.header.stamp = {secs: 0, nsecs: 0};
  c_msg.header.seq = 0;
  publisher->publish(c_msg);
}

template <class T>
void StaticRoomDeviations::publishCloud(T *cloud, ros::Publisher *publisher) const {
  cloud->header.frame_id = map_frame_;
  pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);
  publisher->publish(*cloud);
}

void StaticRoomDeviations::publishPolyhedron(cgal::Polyhedron &P) {
  // use this as long there is no rviz Polyhedron Display for non-triangles
  for (cgal::Polyhedron::Facet_iterator j = P.facets_begin(); j != P.facets_end(); ++j) {
    sleep(5);
    std::cout << "Publish Polyhedron" << std::endl;
    geometry_msgs::PolygonStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time(0);
    cgal::Polyhedron::Halfedge_around_facet_const_circulator hit = j->facet_begin();

    do {
      geometry_msgs::Point32 p;
      p.x = (float)(hit->vertex()->point()).x();
      p.y = (float)(hit->vertex()->point()).y();
      p.z = (float)(hit->vertex()->point()).z();
      msg.polygon.points.push_back(p);
    } while (++hit != j->facet_begin());

    polygon_pub_.publish(msg);
  }
}

void StaticRoomDeviations::publishReconstructedPlanes(const std::vector<reconstructed_plane> &rec_planes, ros::Publisher *publisher) const {
  ColoredPointCloud pointcloud_rgb;
  for (auto plane : rec_planes) {
    ColoredPointCloud pointcloud_plane_rgb;
    pcl::copyPointCloud(plane.pointcloud, pointcloud_plane_rgb);
    uint8_t r = std::rand()%256, g = std::rand()%256, b = std::rand()%256;    
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    for (uint i = 0; i < pointcloud_plane_rgb.points.size(); ++i) {
      pointcloud_plane_rgb.points[i].rgb = *reinterpret_cast<float*>(&rgb);
    }
    pointcloud_rgb += pointcloud_plane_rgb;
  }
  publishCloud<ColoredPointCloud>(&pointcloud_rgb, publisher);
}

}
}
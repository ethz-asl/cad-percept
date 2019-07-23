#include "cpt_selective_icp/mapper.h"

namespace cad_percept {
namespace selective_icp {

Mapper::Mapper(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  : nh_(nh),
    nh_private_(nh_private),
    tf_listener_(ros::Duration(30)) {

  cloud_sub_ = nh_.subscribe(parameters_.scan_topic,
                                     parameters_.input_queue_size,
                                     &Mapper::gotCloud,
                                     this);

  set_ref_srv_ = nh_.advertiseService("set_ref", &Mapper::setReferenceFacets, this);
  reload_icp_config_srv_ = nh_.advertiseService("reload_icp_config", &Mapper::reloadICPConfig, this);
  ref_mesh_pub_ = nh_.advertise<cgal_msgs::ColoredMesh>("ref_mesh", 1, true);
  ref_pc_pub_ = nh_.advertise<PointCloud>("ref_pc", 1, true);
  
  sleep(5); // wait to set up stuff

  loadICPConfig();
  loadReferenceMap();
}

Mapper::~Mapper() {

}

void Mapper::gotCloud(const sensor_msgs::PointCloud2 &cloud_msg_in) {
  std::cout << "Hi" << std::endl;
}

void Mapper::loadReferenceMap() {
  reference_mesh_.init(parameters_.reference_mesh.c_str());
  cgal::Polyhedron P = reference_mesh_.getMesh();

  // Make some checks:
  if (P.is_valid()) {
    std::cout << "P is valid" << std::endl;
  }
  else {
    std::cerr << "P is not valid" << std::endl;
  }
  if (P.is_pure_triangle()) {
    std::cout << "P is pure triangle" << std::endl;
  }
  else {
    std::cerr << "P is not pure triangle" << std::endl;
  }
  if (P.is_closed()) {
    std::cout << "P is closed" << std::endl;
  }
  else {
    std::cerr << "P is not closed" << std::endl;
  }


  // first extract whole ref_pointcloud, ref_pointcloud will be changed later according to references
  std::unordered_set<int> references; // empty
  extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, references, &ref_pointcloud);
}

bool Mapper::setReferenceFacets(cpt_selective_icp::References::Request &req,
                                cpt_selective_icp::References::Response &res) {
  std::unordered_set<int> references;
  for (auto id : req.data) {
    references.insert(id);
  }
  extractReferenceFacets(parameters_.map_sampling_density, reference_mesh_, references, &ref_pointcloud);
  return true;
}

void Mapper::publishReferenceMesh(cgal::MeshModel &reference_mesh, std::unordered_set<int> &references) {
  cgal::Polyhedron P;
  P = reference_mesh.getMesh();
  cgal_msgs::TriangleMesh t_msg;
  cgal_msgs::ColoredMesh c_msg;
  cgal::triangleMeshToMsg(P, &t_msg);
  c_msg.mesh = t_msg;
  c_msg.header.frame_id = parameters_.tf_map_frame;
  c_msg.header.stamp = {secs: 0, nsecs: 0};
  c_msg.header.seq = 0;

  std_msgs::ColorRGBA c;
  
  // set al facet colors to blue
  for (uint i = 0; i < c_msg.mesh.triangles.size(); ++i) {
    c.r = 0.0;
    c.g = 0.0;
    c.b = 1.0;
    c.a = 0.4;
    c_msg.colors.push_back(c);  
  }

  // set reference facets to red
  for (auto reference : references) {
    c.r = 1.0;
    c.g = 0.0;
    c.b = 0.0;
    c.a = 0.4;
    c_msg.colors[reference] = c;
  }
  ref_mesh_pub_.publish(c_msg);
}

template <class T>
void Mapper::publishCloud(T *cloud, ros::Publisher *publisher) const {
  cloud->header.frame_id = parameters_.tf_map_frame;
  pcl_conversions::toPCL(ros::Time(0), cloud->header.stamp);
  publisher->publish(*cloud);
}

void Mapper::extractReferenceFacets(const int density, cgal::MeshModel &reference_mesh, std::unordered_set<int> &references, PointCloud *pointcloud) {
  pointcloud->clear();
  // if reference set is empty, extract whole point cloud
  if (references.empty() == 1) {
    std::cout << "Extract whole point cloud (normal ICP)" << std::endl;
    int n_points = reference_mesh.getArea() * density;
    cgal::sample_pc_from_mesh(reference_mesh.getMesh(), n_points, 0.0, pointcloud, "ref_pointcloud");
    publishReferenceMesh(reference_mesh, references);
  }
  else {
    std::cout << "Extract selected reference point clouds (selective ICP)" << std::endl;

    // sample reference point cloud from mesh

    // use FindAndMergeCoplanarFacets to get coplanar facets
    std::unordered_set<int> references_new;

    std::cout << "Choosen reference facets:" << std::endl;
    std::unordered_set<int>::iterator uitr;
    for (uitr = references.begin(); uitr != references.end(); uitr++) {
      std::cout << *uitr << std::endl;
      reference_mesh.findCoplanarFacets(*uitr, &references_new);
    }

    std::cout << "Computed reference facets:" << std::endl;
    for (uitr = references_new.begin(); uitr != references_new.end(); uitr++) {
      std::cout << *uitr << std::endl;
    }

    publishReferenceMesh(reference_mesh, references_new);

    // create sampled point cloud from reference_new
    
    // generated points
    std::vector<cgal::Point> points;
    // create input triangles
    std::vector<cgal::Triangle> triangles;
    double reference_area = 0;
    cgal::Polyhedron P = reference_mesh.getMesh();
    for (cgal::Polyhedron::Facet_iterator j = P.facets_begin(); j != P.facets_end(); ++j) {
      if (references_new.find(j->id()) != references_new.end()) {
        cgal::Triangle t(j->halfedge()->vertex()->point(),
                         j->halfedge()->next()->vertex()->point(),
                         j->halfedge()->next()->next()->vertex()->point());
        reference_area += CGAL::to_double(sqrt(t.squared_area()));
        triangles.push_back(t);
      }
    }

    // Create the generator, input is the vector of Triangle
    CGAL::Random_points_in_triangles_3<cgal::Point> g(triangles);
    // Get no_of_points random points in cdt
    int no_of_points = reference_area * density;
    CGAL::cpp11::copy_n(g, no_of_points, std::back_inserter(points));
    // Check that we have really created no_of_points.
    assert(points.size() == no_of_points);

    for (auto point : points) {
      pcl::PointXYZ cloudpoint;
      cloudpoint.x = (float)point.x();
      cloudpoint.y = (float)point.y();
      cloudpoint.z = (float)point.z();
      pointcloud->push_back(cloudpoint);
    }
  }
  publishCloud<PointCloud>(pointcloud, &ref_pc_pub_);
}

void Mapper::loadICPConfig() {
  // Load configs.
  std::string config_file_name;
  if (ros::param::get("~icpConfig", config_file_name)) {
    std::ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      icp_.loadFromYaml(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load ICP config from YAML file " << config_file_name);
      icp_.setDefault();
    }
  } else {
    ROS_INFO_STREAM("No ICP config file given, using default");
    icp_.setDefault();
  }

  if (ros::param::get("~inputFiltersConfig", config_file_name)) {
    std::ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      input_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM(
          "Cannot load input filters config from YAML file "
              << config_file_name);
    }
  } else {
    ROS_INFO_STREAM(
        "No input filters config file given, not using these filters");
  }

  if (ros::param::get("~mapPreFiltersConfig", config_file_name)) {
    std::ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      map_pre_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM("Cannot load map pre-filters config from YAML file "
                           << config_file_name);
    }
  } else {
    ROS_INFO_STREAM(
        "No map pre-filters config file given, not using these filters");
  }

  if (ros::param::get("~mapPostFiltersConfig", config_file_name)) {
    std::ifstream ifs(config_file_name.c_str());
    if (ifs.good()) {
      map_post_filters_ = PM::DataPointsFilters(ifs);
    } else {
      ROS_ERROR_STREAM("Cannot load map post-filters config from YAML file "
                           << config_file_name);
    }
  } else {
    ROS_INFO_STREAM(
        "No map post-filters config file given, not using these filters");
  }
}

bool Mapper::reloadICPConfig(std_srvs::Empty::Request &req,
                           std_srvs::Empty::Response &res) {
  loadICPConfig();
  ROS_INFO_STREAM("Parameters reloaded");

  return true;
}

}
}


#if defined (_MSC_VER) && !defined (_WIN64)
#pragma warning(disable:4244) // boost::number_distance::distance()
                              // converts 64 to 32 bits integers
#endif
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Classification.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Real_timer.h>
#include <CGAL/Classification/Point_set_neighborhood.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/OFF_reader.h>

#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_off_points.h>
#include <CGAL/IO/read_ply_points.h>
#include <CGAL/Timer.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/edge_aware_upsample_point_set.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/IO/write_xyz_points.h>
#include <CGAL/pca_estimate_normals.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel          K;
typedef CGAL::Polyhedron_3<K, CGAL::Polyhedron_items_with_id_3>      Polyhedron;

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointVectorPair;
typedef CGAL::Sequential_tag Concurrency_tag;

typedef CGAL::Point_set_3<Point> Point_set;
typedef Kernel::Iso_cuboid_3 Iso_cuboid_3;
typedef Point_set::Point_map Pmap;
typedef Point_set::Property_map<int> Imap;
typedef Point_set::Property_map<unsigned char> UCmap;
namespace Classification = CGAL::Classification;
typedef Classification::Label_handle                                            Label_handle;
typedef Classification::Feature_handle                                          Feature_handle;
typedef Classification::Label_set                                               Label_set;
typedef Classification::Feature_set                                             Feature_set;
typedef Classification::Point_set_feature_generator<Kernel, Point_set, Pmap>    Feature_generator;


#include <iostream>
#include <fstream>
#include <algorithm>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Advancing_front_surface_reconstruction.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Advancing_front_surface_reconstruction<> Reconstruction;
typedef Reconstruction::Triangulation_3 Triangulation_3;
typedef Reconstruction::Triangulation_data_structure_2 TDS_2;
typedef K::Point_3 Point_3;
typedef K::Vector_3 Vector_3;


/*
typedef CGAL::Exact_predicates_inexact_constructions_kernel     Kernel2;
typedef CGAL::Scale_space_surface_reconstruction_3<Kernel2>    Reconstruction;
typedef Kernel2::Point_3 Point3;
typedef Reconstruction::Facet_const_iterator                   Facet_iterator;

typedef CGAL::Surface_mesh<Point> Mesh;
typedef Classification::Face_descriptor_to_center_of_mass_map<Mesh>             Face_point_map;
typedef Classification::Face_descriptor_to_face_descriptor_with_bbox_map<Mesh>  Face_with_bbox_map;
typedef Classification::Mesh_feature_generator<Kernel, Mesh, Face_point_map>    Feature_generator;
typedef Mesh::Property_map<Mesh::Face_index, unsigned char> Mesh_class;
*/

/*
// Mesh based
//Source: https://doc.cgal.org/5.0.4/Classification/index.html
int main (int argc, char** argv){
  const char* input_filename = (argc>1)?argv[1]:"/home/philipp/Schreibtisch/outliers_ros_full.xyz";
  const char* output_filename = (argc>2)?argv[2]:"/home/philipp/Schreibtisch/outliers_ros_UPSAMPLED.xyz";
  // Reads a .xyz point set file in points[], *with normals*.
  std::vector<PointVectorPair> points_up;
  std::ifstream stream(input_filename);
  if (!stream ||
      !CGAL::read_xyz_points(stream,
                             std::back_inserter(points_up),
                             CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
                                 normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())))
  {
    std::cerr << "Error: cannot read file " << input_filename << std::endl;
    return EXIT_FAILURE;
  }
  //Algorithm parameters
  const double sharpness_angle = 40;   // control sharpness of the result.
  const double edge_sensitivity = 0.2;    // higher values will sample more points near the edges
  const double neighbor_radius = 0.5;  // initial size of neighborhood.
  const std::size_t number_of_output_points = points_up.size() * 2;
  //Run algorithm
  CGAL::edge_aware_upsample_point_set<Concurrency_tag>(
      points_up,
      std::back_inserter(points_up),
      CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
          normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()).
          sharpness_angle(sharpness_angle).
          edge_sensitivity(edge_sensitivity).
          neighbor_radius(neighbor_radius).
          number_of_output_points(number_of_output_points));
  // Saves point set.
  std::ofstream out2(output_filename);
  out2.precision(17);
  if (!out2 ||
      !CGAL::write_xyz_points(
          out2, points_up,
          CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
              normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>())))
  {
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;

  std::vector<Point3> points;
  std::ifstream in("/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/dataPreperation/prediction_data.ply");
  std::cerr << "Reading " << std::flush;
  if( !in || !CGAL::read_ply_points( in, std::back_inserter( points ) ) ) {
    std::cerr << "Error: cannot read file" << std::endl;
    return EXIT_FAILURE;
  }

  std::cerr << "done: " << points.size() << " points." << std::endl;
  std::cerr << "Reconstruction ";
  CGAL::Timer t;
  t.start();
  // Construct the mesh in a scale space.
  Reconstruction reconstruct (points.begin(), points.end());
  reconstruct.increase_scale(4);
  reconstruct.reconstruct_surface();
  std::cerr << "done in " << t.time() << " sec." << std::endl;
  t.reset();
  std::ofstream out ("out.off");
  out << reconstruct;
  std::cerr << "Writing result in " << t.time() << " sec." << std::endl;
  std::cerr << "Done." << std::endl;


  CGAL::Timer t;
  t.start();
  //std::string filename = "/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/ClassPrediction/cmake-build-debug/out.off";
  std::string filename = "/home/philipp/Schreibtisch/out2.off";

  //Source: https://doc.cgal.org/5.0.4/Polygon_mesh_processing/index.html#OrientingPolygonMeshes
  std::ifstream in(filename);
  std::vector<K::Point_3> points;
  std::vector<std::vector<std::size_t> > polygons;
  if(!in || !CGAL::read_OFF(in, points, polygons) || points.empty())
  {
    std::cerr << "Cannot open file " << std::endl;
    return EXIT_FAILURE;
  }
  CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);
  Polyhedron mesh_temp;
  CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, mesh_temp);
  // Number the faces because 'orient_to_bound_a_volume' needs a face <--> index map
  int index = 0;
  for(Polyhedron::Face_iterator fb=mesh_temp.facets_begin(), fe=mesh_temp.facets_end(); fb!=fe; ++fb)
    fb->id() = index++;
  if(CGAL::is_closed(mesh_temp))
    CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(mesh_temp);
  std::ofstream out("tet-oriented1.off");
  out << mesh_temp;
  out.close();

  //TODO: Error when reading out.off
  std::string filename2 = "/home/philipp/Schreibtisch/tet-oriented1.off";
  std::ifstream i2(filename2.c_str());
  Mesh mesh;
  i2 >> mesh;


  std::cout << "Empty mesh?: " << mesh.is_empty() << std::endl;
  std::cout << "Number of faces: " << mesh.number_of_faces() << std::endl;

  std::cerr << "1" << std::endl;
  Feature_set features;
  Face_point_map face_point_map (&mesh);
  std::size_t number_of_scales = 5;
  Feature_generator generator (mesh, face_point_map, number_of_scales);

  std::cerr << "2" << std::endl;
  generator.generate_point_based_features (features);
  generator.generate_face_based_features (features);

  std::cerr << "3" << std::endl;
  // Add types
  Label_set labels;
  Label_handle wall = labels.add ("wall");
  Label_handle ceiling = labels.add ("ceiling");
  Label_handle beam = labels.add ("beam");
  Label_handle column = labels.add ("column");
  Label_handle floor = labels.add ("floor");
  Label_handle clutter = labels.add ("clutter");


  std::vector<int> label_indices(mesh.number_of_faces(), -1);
  std::cerr << "Using ETHZ Random Forest Classifier" << std::endl;
  Classification::ETHZ::Random_forest_classifier classifier (labels, features);

  std::ifstream input;
  input.open("/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/cmake-build-debug/ethz_random_forest.bin", std::ios::in | std::ios::binary);
  classifier.load_configuration(input);

  t.reset();
  t.start();
  Classification::classify_with_graphcut<CGAL::Sequential_tag>
      (mesh.faces(), Face_with_bbox_map(&mesh), labels, classifier,
       generator.neighborhood().n_ring_neighbor_query(2),
       0.2f, 1, label_indices);
  t.stop();
  std::cerr << "Classification with graphcut done in " << t.time() << " second(s)" << std::endl;

  //Classification::classify<CGAL::Sequential_tag> (pts, labels, classifier, label_indices);


  // Color point set according to class
  Mesh_class red = mesh.add_property_map<Mesh::Face_index, unsigned char>("red", 0).first;
  Mesh_class green = mesh.add_property_map<Mesh::Face_index, unsigned char>("green", 0).first;
  Mesh_class blue = mesh.add_property_map<Mesh::Face_index, unsigned char>("blue", 0).first;


  for (std::size_t i = 0; i < label_indices.size(); ++ i)
  {
    Label_handle label = labels[label_indices[i]];
    Mesh::Face_index idx(i);

    if (label == beam)
    {
      //#magenta -> beam color = np.array([255,255,0])
      red[idx] = 255; green[idx] = 255; blue[idx] =   0;
    }
    else if (label == ceiling)
    {
      //# green -> ceiling color = np.array([0,255,00])
      red[idx] =   0; green[idx] = 255; blue[idx] =  0;
    }
    else if (label == clutter)
    {
      //# black -> clutter color = np.array([50,50,50])
      red[idx] =   50; green[idx] = 50; blue[idx] =  50;
    }
    else if (label == column)
    {
      //# pink -> column color = np.array([255,0,255])
      red[idx] = 255; green[idx] =   0; blue[idx] = 255;
    }
    else if (label == floor)
    {
      //# blue -> floor color = np.array([0, 0, 255])
      red[idx] = 0; green[idx] =   0; blue[idx] = 255;
    }
    else if (label == wall)
    {
      //# red -> wall color = np.array([0, 255, 255])
      red[idx] =   0; green[idx] = 255; blue[idx] =  255;
    }
  }

  // Write result
  std::ofstream f ("prediction.off");
  f.precision(18);
  f << mesh;
  std::cerr << "All done" << std::endl;
  return EXIT_SUCCESS;
}
*/



// Point based
//Source: https://doc.cgal.org/5.0.4/Classification/index.html
// https://doc.cgal.org/latest/Manual/tuto_reconstruction.html
int main (int argc, char** argv) {

  std::string filename = "/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/dataPreperation/prediction_data.ply";
  std::ifstream in (filename.c_str(), std::ios::binary);
  Point_set pts;
  std::cerr << "Reading input" << std::endl;
  in >> pts;

  Feature_set features;
  std::cerr << "Generating features" << std::endl;
  CGAL::Real_timer t;
  t.start();
  Feature_generator generator (pts, pts.point_map(),
                               5);  // using 5 scales

#ifdef CGAL_LINKED_WITH_TBB
  features.begin_parallel_additions();
#endif
  generator.generate_point_based_features (features);
#ifdef CGAL_LINKED_WITH_TBB
  features.end_parallel_additions();
#endif
  t.stop();

  std::cerr << "Done in " << t.time() << " second(s)" << std::endl;
  // Add types
  Label_set labels;
  Label_handle wall = labels.add ("wall");
  Label_handle ceiling = labels.add ("ceiling");
  Label_handle beam = labels.add ("beam");
  Label_handle column = labels.add ("column");
  Label_handle floor = labels.add ("floor");
  Label_handle clutter = labels.add ("clutter");

  std::vector<int> label_indices(pts.size(), -1);
  std::cerr << "Using ETHZ Random Forest Classifier" << std::endl;
  Classification::ETHZ::Random_forest_classifier classifier (labels, features);

  std::ifstream input;
  input.open("/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/cmake-build-debug/ethz_random_forest_random.bin", std::ios::in | std::ios::binary);
  classifier.load_configuration(input);

  t.reset();
  t.start();
  Classification::classify_with_graphcut<CGAL::Sequential_tag>
      (pts, pts.point_map(), labels, classifier,
       generator.neighborhood().k_neighbor_query(12),
       0.2f, 1, label_indices);
  t.stop();
  std::cerr << "Classification with graphcut done in " << t.time() << " second(s)" << std::endl;

  //Classification::classify<CGAL::Sequential_tag> (pts, labels, classifier, label_indices);


  // Color point set according to class
  UCmap red = pts.add_property_map<unsigned char>("red", 0).first;
  UCmap green = pts.add_property_map<unsigned char>("green", 0).first;
  UCmap blue = pts.add_property_map<unsigned char>("blue", 0).first;


  for (std::size_t i = 0; i < label_indices.size(); ++ i)
  {
    Label_handle label = labels[label_indices[i]];

    if (label == beam)
    {
      //#magenta -> beam color = np.array([255,255,0])
      red[i] = 255; green[i] = 255; blue[i] =   0;
    }
    else if (label == ceiling)
    {
      //# green -> ceiling color = np.array([0,255,00])
      red[i] =   0; green[i] = 255; blue[i] =  0;
    }
    else if (label == clutter)
    {
      //# black -> clutter color = np.array([50,50,50])
      red[i] =   50; green[i] = 50; blue[i] =  50;
    }
    else if (label == column)
    {
      //# pink -> column color = np.array([255,0,255])
      red[i] = 255; green[i] =   0; blue[i] = 255;
    }
    else if (label == floor)
    {
      //# blue -> floor color = np.array([0, 0, 255])
      red[i] = 0; green[i] =   0; blue[i] = 255;
    }
    else if (label == wall)
    {
      //# red -> wall color = np.array([0, 255, 255])
      red[i] =   0; green[i] = 255; blue[i] =  255;
    }
  }

  // Write result
  std::ofstream f ("cla_clusters_random.ply");
  f.precision(18);
  f << pts;
  std::cerr << "All done" << std::endl;
  return EXIT_SUCCESS;
}
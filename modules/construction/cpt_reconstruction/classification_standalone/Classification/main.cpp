#if defined (_MSC_VER) && !defined (_WIN64)
#pragma warning(disable:4244) // boost::number_distance::distance()
// converts 64 to 32 bits integers
#endif
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Classification.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Real_timer.h>
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef CGAL::Surface_mesh<Point> Mesh;
typedef CGAL::Point_set_3<Point> Point_set;
typedef Kernel::Iso_cuboid_3 Iso_cuboid_3;

typedef Point_set::Point_map Pmap;
typedef Point_set::Property_map<int> Imap;
typedef Point_set::Property_map<unsigned char> UCmap;

namespace Classification = CGAL::Classification;
typedef Classification::Label_handle Label_handle;
typedef Classification::Feature_handle Feature_handle;
typedef Classification::Label_set Label_set;
typedef Classification::Feature_set Feature_set;
//typedef Classification::Point_set_feature_generator<Kernel, Point_set, Pmap>    Feature_generator;

typedef Classification::Face_descriptor_to_center_of_mass_map<Mesh> Face_point_map;
typedef Classification::Face_descriptor_to_face_descriptor_with_bbox_map<Mesh> Face_with_bbox_map;
typedef Classification::Mesh_feature_generator<Kernel, Mesh, Face_point_map> Feature_generator;

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>


//MeshBased
int main(int argc, char **argv) {

  //Read in points with lables
  /*
  std::string filename_points =
      "/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/dataPreperation/training_data_area1_random.ply";
  std::ifstream in_points(filename_points.c_str(), std::ios::binary);
  Point_set pts;
  in_points >> pts;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::string filename_points2 =
      "/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/dataPreperation/training_data_area1_random_float.ply";
  pcl::io::loadPLYFile<pcl::PointXYZ> (filename_points2, *cloud);
  std::cout << "PLY Size" << cloud->size() << std::endl;


  Imap label_map;
  bool lm_found = false;
  boost::tie(label_map, lm_found) = pts.property_map<int>("label");
  if (!lm_found) {
    std::cerr << "Error: \"label\" property not found in input file." << std::endl;
    return EXIT_FAILURE;
  }
  std::vector<int> ground_truth_points;
  ground_truth_points.reserve(pts.size());
  std::copy(pts.range(label_map).begin(), pts.range(label_map).end(),
            std::back_inserter(ground_truth_points));


  std::vector<int> ground_truth_points_down;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < cloud->size(); i += 5){
    cloud_down->push_back((*cloud)[i]);
    ground_truth_points_down.push_back(ground_truth_points.at(i));
  }
  pcl::search::KdTree<pcl::PointXYZ> kd_tree;
  kd_tree.setInputCloud(cloud_down);
  */
  //Read in mesh
  std::string filename_mesh =
      "/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/ClassPredicitonReconstruction/cmake-build-debug/training_data_cla.off";
  std::ifstream in_mesh(filename_mesh.c_str(), std::ios::binary);
  Mesh mesh;
  CGAL::read_off(in_mesh, mesh);

  std::cout << mesh.num_vertices() << std::endl;
  std::cout << mesh.num_faces() << std::endl;

  /*
  std::vector<int> mesh_lables;
  //Source: https://stackoverflow.com/questions/48104677/cgal-iterate-over-all-vertices-of-all-faces
  std::vector<int> idx_seach(1);
  std::vector<float> dist_search(1);
  int i = 0;
  BOOST_FOREACH(Mesh::Face_index f, mesh.faces()) {
    int j = 0;
    CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
    Eigen::Vector3d mean_point(0, 0, 0);

    for (boost::tie(vbegin, vend) = vertices_around_face(mesh.halfedge(f), mesh); vbegin != vend; ++vbegin) {
      Point p1 = mesh.point(*vbegin);
      Eigen::Vector3d e1(p1.x(), p1.y(), p1.z());
      mean_point += e1;
      j++;
    }
    mean_point /= j;

    idx_seach.clear();
    dist_search.clear();

    pcl::PointXYZ p_s(mean_point.x(), mean_point.y(), mean_point.z());
    kd_tree.nearestKSearch(p_s, 1, idx_seach, dist_search);
    int idx = idx_seach.at(0);
    mesh_lables.push_back(ground_truth_points_down.at(idx));

    i++;
    std::cout << ((double)i / (double)mesh.num_faces()) << std::endl;
  }

  std::cout << "mesh_lables" << mesh_lables.size() << std::endl;
  std::cout << "number faces" << mesh.num_faces() << std::endl;
  */
  std::size_t number_of_scales = 5;
  Face_point_map face_point_map(&mesh);
  Feature_generator generator (mesh, face_point_map, number_of_scales);

  Feature_set features;
  generator.generate_point_based_features (features);
  generator.generate_face_based_features (features);

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


  //std::cerr << "Training" << std::endl;
  //classifier.train (mesh_lables);

  //std::ofstream fconfig ("/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/cmake-build-debug/mesh_ethz_random_forest_random.bin", std::ios_base::binary);
  //classifier.save_configuration(fconfig);
  //std::cerr << "Saved config" << std::endl;


  std::ifstream in_config ("/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/cmake-build-debug/mesh_ethz_random_forest_random.bin", std::ios_base::in | std::ios_base::binary);
  classifier.load_configuration(in_config);

  Classification::classify_with_graphcut<CGAL::Sequential_tag>
      (mesh.faces(), Face_with_bbox_map(&mesh), labels, classifier,
       generator.neighborhood().n_ring_neighbor_query(2),
       0.2f, 1, label_indices);

  std::cout << "classification done" << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result (new pcl::PointCloud<pcl::PointXYZRGB>());
  int face_idx = 0;
  BOOST_FOREACH(Mesh::Face_index f, mesh.faces()) {
          int j = 0;

          int red = 0;
          int green = 0;
          int blue = 0;
          Label_handle label = labels[label_indices[face_idx]];
          if (label == beam)
          {
            //#magenta -> beam color = np.array([255,255,0])
            red = 255; green = 255; blue =   0;
          }
          else if (label == ceiling)
          {
            //# green -> ceiling color = np.array([0,255,00])
            red =   0; green = 255; blue =  0;
          }
          else if (label == clutter)
          {
            //# black -> clutter color = np.array([50,50,50])
            red =   50; green = 50; blue =  50;
          }
          else if (label == column)
          {
            //# pink -> column color = np.array([255,0,255])
            red = 255; green =   0; blue = 255;
          }
          else if (label == floor)
          {
            //# blue -> floor color = np.array([0, 0, 255])
            red = 0; green =   0; blue = 255;
          }
          else if (label == wall)
          {
            //# red -> wall color = np.array([0, 255, 255])
            red =   0; green = 255; blue =  255;
          } else {
            std::cout << "lable not found" << std::endl;
          }

          CGAL::Vertex_around_face_iterator<Mesh> vbegin, vend;
          for (boost::tie(vbegin, vend) = vertices_around_face(mesh.halfedge(f), mesh); vbegin != vend; ++vbegin) {
            Point p1 = mesh.point(*vbegin);
            pcl::PointXYZRGB p_rgb;
            p_rgb.x = (float)p1.x();
            p_rgb.y = (float)p1.y();
            p_rgb.z = (float)p1.z();
            p_rgb.r = (std::uint8_t)red;
            p_rgb.g = (std::uint8_t)green;
            p_rgb.b = (std::uint8_t)blue;
            result->push_back(p_rgb);
          }
          face_idx++;
        }

  // Write result
  pcl::io::savePLYFileBinary("mesh_classification_cla_random.ply", *result);
  std::cerr << "All done" << std::endl;

  return EXIT_SUCCESS;
  return 0;
}

/*
//PointBased
//Source: https://doc.cgal.org/5.0.4/Classification/index.html
int main2 (int argc, char** argv)
{
  std::string filename = "/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/dataPreperation/training_data_area1_random.ply";
  std::ifstream in (filename.c_str(), std::ios::binary);
  Point_set pts;
  std::cerr << "Reading input" << std::endl;
  in >> pts;
  Imap label_map;
  bool lm_found = false;
  boost::tie (label_map, lm_found) = pts.property_map<int> ("label");
  if (!lm_found)
  {
    std::cerr << "Error: \"label\" property not found in input file." << std::endl;
    return EXIT_FAILURE;
  }
  std::vector<int> ground_truth;
  ground_truth.reserve (pts.size());
  std::copy (pts.range(label_map).begin(), pts.range(label_map).end(),
             std::back_inserter (ground_truth));
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
  // lables = ["beam", "board", "bookcase", "ceiling", "chair", "clutter", "column", "door", "floor", "sofa", "table", "wall",  "window"]

  Label_handle wall = labels.add ("wall");
  Label_handle ceiling = labels.add ("ceiling");
  Label_handle beam = labels.add ("beam");
  Label_handle column = labels.add ("column");
  Label_handle floor = labels.add ("floor");
  Label_handle clutter = labels.add ("clutter");

  std::vector<int> label_indices(pts.size(), -1);
  std::cerr << "Using ETHZ Random Forest Classifier" << std::endl;
  Classification::ETHZ::Random_forest_classifier classifier (labels, features);

  std::cerr << "Training" << std::endl;
  t.reset();
  t.start();
  classifier.train (ground_truth);
  t.stop();
  std::cerr << "Done in " << t.time() << " second(s)" << std::endl;

  std::ofstream fconfig ("/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/cmake-build-debug/ethz_random_forest_random.bin", std::ios_base::binary);
  classifier.save_configuration(fconfig);
  std::cerr << "Saved config" << std::endl;

  t.reset();
  t.start();
  Classification::classify_with_graphcut<CGAL::Sequential_tag>
      (pts, pts.point_map(), labels, classifier,
       generator.neighborhood().k_neighbor_query(12),
       0.2f, 1, label_indices);


  t.stop();
  std::cerr << "Classification with graphcut done in " << t.time() << " second(s)" << std::endl;
  std::cerr << "Precision, recall, F1 scores and IoU:" << std::endl;
  Classification::Evaluation evaluation (labels, ground_truth, label_indices);
  for (std::size_t i = 0; i < labels.size(); ++ i)
  {
    std::cerr << " * " << labels[i]->name() << ": "
              << evaluation.precision(labels[i]) << " ; "
              << evaluation.recall(labels[i]) << " ; "
              << evaluation.f1_score(labels[i]) << " ; "
              << evaluation.intersection_over_union(labels[i]) << std::endl;
  }
  std::cerr << "Accuracy = " << evaluation.accuracy() << std::endl
            << "Mean F1 score = " << evaluation.mean_f1_score() << std::endl
            << "Mean IoU = " << evaluation.mean_intersection_over_union() << std::endl;
  // Color point set according to class
  UCmap red = pts.add_property_map<unsigned char>("red", 0).first;
  UCmap green = pts.add_property_map<unsigned char>("green", 0).first;
  UCmap blue = pts.add_property_map<unsigned char>("blue", 0).first;

  //lables = ["beam", "board", "bookcase", "ceiling", "chair", "clutter", "column", "door", "floor", "sofa", "table", "wall",  "window"]
  for (std::size_t i = 0; i < label_indices.size(); ++ i)
  {
    label_map[i] = label_indices[i]; // update label map with computed classification
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
  std::ofstream f ("classification_random.ply");
  f.precision(18);
  f << pts;
  std::cerr << "All done" << std::endl;
  return EXIT_SUCCESS;
} */
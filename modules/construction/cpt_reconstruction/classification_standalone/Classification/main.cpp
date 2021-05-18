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
#include <CGAL/Real_timer.h>
typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
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


//Source: https://doc.cgal.org/5.0.4/Classification/index.html
int main (int argc, char** argv)
{
  std::string filename = "/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/dataPreperation/training_data_area1.ply";
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
                               2);  // using 5 scales

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
  if (true){
    std::cerr << "Training" << std::endl;
    t.reset();
    t.start();
    classifier.train (ground_truth);
    t.stop();
    std::cerr << "Done in " << t.time() << " second(s)" << std::endl;

    std::ofstream fconfig ("/home/philipp/reconstruction_ws/src/cad-percept/modules/construction/cpt_reconstruction/classification_standalone/Classification/cmake-build-debug/ethz_random_forest.bin", std::ios_base::binary);
    classifier.save_configuration(fconfig);
    std::cerr << "Saved config" << std::endl;
  } else {
    std::ifstream input;
    input.open("ethz_random_forest.bin", std::ios::in | std::ios::binary);
    classifier.load_configuration(input);
  }

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
  std::ofstream f ("classification.ply");
  f.precision(18);
  f << pts;
  std::cerr << "All done" << std::endl;
  return EXIT_SUCCESS;
}
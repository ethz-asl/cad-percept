#include <cpt_meshing/meshing_node.h>
using namespace std;

int main(int argc, char** argv) {

  ros::init(argc, argv, "cpt_meshing_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  cad_percept::meshing::CadPerceptMeshingNode meshing_node(nh, nh_private);

  ros::spin();

  return 0;
}
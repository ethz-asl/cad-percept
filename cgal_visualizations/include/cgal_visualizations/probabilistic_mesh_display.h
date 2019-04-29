#ifndef CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_DISPLAY_H
#define CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_DISPLAY_H

#include <Eigen/Dense>
#include <rviz/message_filter_display.h>
#include <cgal_msgs/ProbabilisticMesh.h>
#include "probabilistic_mesh_visual.h"

namespace cad_percept {

namespace visualizations {

class ProbabilisticMeshVisual;  // Forward definition

class ProbabilisticMeshDisplay
    : public rviz::MessageFilterDisplay<cgal_msgs::ProbabilisticMesh> {
 Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ProbabilisticMeshDisplay() {}
  virtual ~ProbabilisticMeshDisplay();

  void initProperties();

 private Q_SLOTS:
  // Property Callbacks
  void backfaceCullingPropertyChanged();

 protected:
  virtual void onInitialize();
  virtual void reset();

 private:
  void processMessage(const cgal_msgs::ProbabilisticMesh::ConstPtr& msg);

  struct {
    rviz::BoolProperty* BackfaceCulling;
  } properties_;

  std::unique_ptr<ProbabilisticMeshVisual> visual_;
};

}  // namespace visualizations
}  // namespace cad_percept

#endif  // CGAL_VISUALIZATIONS_PROBABILISTIC_MESH_DISPLAY_H

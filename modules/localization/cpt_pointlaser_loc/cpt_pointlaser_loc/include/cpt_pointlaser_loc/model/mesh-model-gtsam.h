#ifndef CPT_POINTLASER_LOC_MODEL_MESH_MODEL_GTSAM_H_
#define CPT_POINTLASER_LOC_MODEL_MESH_MODEL_GTSAM_H_

#include <cgal_definitions/mesh_model.h>
#include <glog/logging.h>

namespace gtsam {
template <>
struct traits<cad_percept::cgal::MeshModel::Ptr> {
  // The dimension of the manifold.
  enum { dimension = 1 };

  typedef cad_percept::cgal::MeshModel::Ptr type;
  // the "vector" typedef is used by gtsam.
  typedef Eigen::Matrix<double, dimension, 1> vector;

  // Print the type.
  static void Print(const type& T, const std::string& str) {
    std::cout << "MeshModel Expression Fake," << str << std::endl;
  }

  static void Print(const type& T) { Print(T, ""); }

  // Check the equality of two values.
  static bool Equals(const type& T1, const type& T2, double tol) {
    CHECK(false) << "MeshModel Equals not implemented.";
    return false;
  }

  static bool Equals(const type& T1, const type& T2) {
    CHECK(false) << "MeshModel Equals not implemented.";
    return false;
  }

  static vector Local(const type& origin, const type& other) {
    CHECK(false) << "MeshModel Local not implemented.";
    return vector::Zero();
  }
  static type Retract(const type& origin, const vector& d) {
    CHECK(false) << "MeshModel Retract not implemented.";
    return origin;
  }
  static int GetDimension(const type& /* origin */) { return dimension; }
};  // traits
}  // namespace gtsam
#endif  // CPT_POINTLASER_LOC_MODEL_MESH_MODEL_GTSAM_H_

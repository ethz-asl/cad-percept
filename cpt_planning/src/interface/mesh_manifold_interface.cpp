#include <cpt_planning/interface/mesh_manifold_interface.h>

namespace cad_percept {
namespace planning {

Eigen::Matrix3d MeshManifoldInterface::J(const Eigen::Vector3d p_manifold) {
  // look up triangle

  Eigen::Vector3d A_xyz, B_xyz, C_xyz;  // triangle vertices in xyz
  Eigen::Vector3d A_uv, B_uv, C_uv;     // triangle vertices in uv

  Eigen::Vector3d AB_xyz, AC_xyz, N_xyz;

  AB_xyz = B_xyz - A_xyz;  // "-A+B"
  AC_xyz = C_xyz - A_xyz;  // "-A+C"
  N_xyz = AB_xyz.cross(AC_xyz).normalized();

  Eigen::Vector3d I, K;  // vectors do build up the derivatve.
  Eigen::Vector3d part_u, part_v, part_h;
  double d00, d11, d01;
  double denom;

  // constants
  d00 = AB_xyz.dot(AB_xyz);
  d01 = AB_xyz.dot(AC_xyz);
  d11 = AC_xyz.dot(AC_xyz);
  denom = d00 * d11 - d01 * d01;

  I = (d00 * AC_xyz - d01 * AB_xyz) / denom;
  K = (d11 * AB_xyz - d01 * AC_xyz) / denom;

  // u/x, u/y, u/z
  part_u = B_uv[0] * K + C_uv[0] * I - A_uv[0] * (I + K);

  // v/x, v/y, v/z
  part_v = B_uv[1] * K + C_uv[1] * I - A_uv[1] * (I + K);

  // h/x, h/y, h/z
  part_h = N_xyz;

  /*        | u/x  u/y  u/z |
   * J_mtx =| v/x  v/y  v/z |
   *        | h/x  h/y  h/z |
   */
  Eigen::Matrix3d J_mtx;
  J_mtx.row(0) = part_u;
  J_mtx.row(1) = part_v;
  J_mtx.row(2) = part_h;

  return J_mtx;
}

}  // namespace planning
}  // namespace cad_percept
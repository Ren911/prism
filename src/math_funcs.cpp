#include "math_funcs.h"
#include <eigen3/Eigen/src/Core/Matrix.h>

namespace math {

Eigen::Vector2d to2D(const Eigen::Vector3d &point3d) {
  return {point3d.x(), point3d.y()};
}

Eigen::Vector3d to3D(const Eigen::Vector2d &point2d) {
  return {point2d.x(), point2d.y(), 0};
}

bool isPointsInPlane(const std::vector<Eigen::Vector3d> &points) {
  int n_points = points.size();
  Eigen::Vector3d a = points.at(1) - points.at(0);
  Eigen::Vector3d b = points.at(2) - points.at(0);
  for (int i = 2; i < n_points; ++i) {
    Eigen::Vector3d c = points.at(i) - points.at(0);
    if (abs(a.dot(b.cross(c))) > 1e-4)
      return false;
    b = c;
  }
  return true;
}

Eigen::Matrix3d getGlobalToLocalRotation(const Eigen::Vector3d &vec_global,
                                         const Eigen::Vector3d &vec_local) {
  return Eigen::Quaterniond::FromTwoVectors(vec_global, vec_local)
      .toRotationMatrix();
}

Eigen::Vector3d getNormal(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                          const Eigen::Vector3d &p2) {
  Eigen::Vector3d v1 = p1 - p0;
  Eigen::Vector3d v2 = p2 - p0;
  return v1.cross(v2).normalized();
}

Eigen::Vector3d transformToGlobal(const Eigen::Vector3d &point_local,
                                  const Eigen::Matrix3d &R,
                                  const Eigen::Vector3d &translation_global) {
  return R.transpose() * point_local + translation_global;
}

Eigen::Vector3d transformToLocal(const Eigen::Vector3d &point_global,
                                 const Eigen::Matrix3d &R,
                                 const Eigen::Vector3d &translation_global) {
  return R * (point_global - translation_global);
}

} // namespace math

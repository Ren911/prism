#include "polygon3d.h"
#include "math_funcs.h"
#include <stdexcept>

Polygon3d::Polygon3d(const std::vector<Eigen::Vector3d> &points)
    : points_(points) {
  if (!math::isPointsInPlane(points))
    throw std::invalid_argument("Points for 3d polygon are not in one plane");
  ;
};

Eigen::Matrix3d Polygon3d::getRotationToLocal() const {
  Eigen::Vector3d global_normal =
      math::getNormal(points_.at(0), points_.at(1), points_.at(2));
  Eigen::Vector3d local_normal = Eigen::Vector3d::UnitZ();
  return math::getGlobalToLocalRotation(global_normal, local_normal);
}

Polygon
Polygon3d::getLocalPolygon2d(const Eigen::Matrix3d &rot_matrix,
                             const Eigen::Vector3d &translation_global) const {
  std::vector<Eigen::Vector2d> points2d_local;
  points2d_local.reserve(points_.size());
  std::transform(points_.begin(), points_.end(),
                 std::back_inserter(points2d_local), [&](const auto &point) {
                   Eigen::Vector3d point_local = math::transformToLocal(
                       point, rot_matrix, translation_global);
                   return math::to2D(point_local);
                 });
  return Polygon(points2d_local);
}

Eigen::Vector3d Polygon3d::center() const {
  Eigen::Matrix3d rot_matrix = getRotationToLocal();
  const Eigen::Vector3d &translation_global = points_.at(0);
  Eigen::Vector2d center2d_local =
      getLocalPolygon2d(rot_matrix, translation_global).center();
  Eigen::Vector3d center3d_local = math::to3D(center2d_local);
  Eigen::Vector3d center3d_global =
      math::transformToGlobal(center3d_local, rot_matrix, translation_global);
  return center3d_global;
}

#ifndef MATH_FUNCS_H
#define MATH_FUNCS_H

#include <eigen3/Eigen/Dense>
#include <vector>

namespace math {
// Project point3d on Z-plane (remove z-dim)
Eigen::Vector2d to2D(const Eigen::Vector3d &point3d);

// Add 3rd dimension as z = 0
Eigen::Vector3d to3D(const Eigen::Vector2d &point2d);

// Return unit normal vector from 3 points in plane
Eigen::Vector3d getNormal(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1,
                          const Eigen::Vector3d &p2);

bool isPointsInPlane(const std::vector<Eigen::Vector3d> &points) ;

// Return Rotation matrix `R` such that  local_vec = R*global_vec
// Warning: vectors should have equal length
Eigen::Matrix3d getGlobalToLocalRotation(const Eigen::Vector3d &vec_global,
                                         const Eigen::Vector3d &vec_local);
//Transforms local point to global point
// vec_global = R_inverse*vec_local;
// point_global = R_inverse*point_local + global_translation
// translation_global = (origin_of_local) in global coordinates
// R_inverse = R^T as R = orthogonal rotation matrix
Eigen::Vector3d transformToGlobal(const Eigen::Vector3d &point_local,
                                  const Eigen::Matrix3d &R,
                                  const Eigen::Vector3d &translation_global);
//Transforms global point to local point
// vec_local = R*vec_global;
// point_local = R*(point_global - global_translation)
Eigen::Vector3d transformToLocal(const Eigen::Vector3d &point_global,
                                 const Eigen::Matrix3d &R,
                                 const Eigen::Vector3d &translation_global);
} //namespace math

#endif // MATH_FUNCS_H

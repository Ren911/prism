#include "polygon3d.h"
#include "prism.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

// https://e-maxx.ru/algo/gravity_center

int main() {
  // constructs plane
  Eigen::Vector3d plane_normal = {0, 0, 1};
  Eigen::Vector3d plane_point = {0, 0, -10};
  plane_normal.normalize();
  Plane3d plane(plane_normal, plane_point);
  // constructs prism
  std::vector<Eigen::Vector3d> base_points = {
      {0, 0, 0}, {0, 1, 0}, {1, 1, 1}, {1, 0, 1}};
  Polygon3d polygon3d{base_points};         // for one base of prism
  Eigen::Vector3d shift_vector = {1, 1, 1}; // shift for second parallel base
  Prism prism(polygon3d, shift_vector);
  std::cout << "Prism 1-st base " << base_points.size() << "points-polygon \n";
  for (const auto &point : base_points) {
    std::cout << point.transpose() << "\n";
  }
  std::cout << "Prism 2-st base " << base_points.size() << "points-polygon \n";
  for (const auto &point : base_points) {
    std::cout << (point + shift_vector).transpose() << "\n";
  }
  std::cout << "Prism shift vector between bases:\n"
            << shift_vector.transpose() << "\n";
  std::cout << "Prism center of mass:\n" << prism.center().transpose() << "\n";

  std::cout << "Plane " << plane.coeffs()(0) << "x + " << plane.coeffs()(1)
            << "y + " << plane.coeffs()(2) << "z + " << plane.coeffs()(3)
            << " = 0 "
            << "\n";
  std::cout << "Prism distance to plane: " << prism.getDistanceToPlane(plane)
            << "\n";

  return 0;
}

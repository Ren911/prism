#include "polygon.h"
#include "triangle.h"

Polygon::Polygon(const std::vector<Eigen::Vector2d> &points)
    : points_(points){};

Eigen::Vector2d Polygon::center() const {
  double total_area = 0.0;
  Eigen::Vector2d weighted_centers_sum = Eigen::Vector2d::Zero();
  const auto &reference_point = points_.at(0);
  int n = points_.size();
  for (int i = 1; i < n - 1; i++) {
    Triangle triangle{reference_point, points_.at(i), points_.at(i + 1)};
    double area = triangle.orientedArea();
    weighted_centers_sum += area * triangle.center();
    total_area += area;
  }
  return weighted_centers_sum / total_area;
}

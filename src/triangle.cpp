#include "triangle.h"

Triangle::Triangle(const Eigen::Vector2d &p0, const Eigen::Vector2d &p1,
                   const Eigen::Vector2d &p2)
    : p0_(p0), p1_(p1), p2_(p2){};

Eigen::Vector2d Triangle::center() const { return (p0_ + p1_ + p2_) / 3.0; };

// https://e-maxx.ru/algo/oriented_area
double Triangle::orientedArea() const {
  Eigen::Vector2d a = p1_ - p0_;
  Eigen::Vector2d b = p2_ - p0_;
  return 0.5 * (a.x() * b.y() - a.y() * b.x());
}

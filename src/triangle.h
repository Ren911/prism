#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <eigen3/Eigen/Dense>

class Triangle
{
public:
    Triangle(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
    Eigen::Vector2d center() const;
    double orientedArea() const;

private:
    Eigen::Vector2d p0_;
    Eigen::Vector2d p1_;
    Eigen::Vector2d p2_;
};
#endif // TRIANGLE_H

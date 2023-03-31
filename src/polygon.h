#ifndef POLYGON_H
#define POLYGON_H

#include <eigen3/Eigen/Dense>
#include <vector>
class Polygon
{
public:
    //Creates n-polygon from vector of points {p0, p1, ..., pn-1}
    Polygon(const std::vector<Eigen::Vector2d>& points);

    //Returns center of mass coordinate of polygon
    //https://e-maxx.ru/algo/gravity_center
    //Algorithm based on sum of weighted oriented areas of triangles
    //Polygon can be nonconvex
    Eigen::Vector2d center() const;

private:
    std::vector<Eigen::Vector2d> points_;
};


#endif // POLYGON_H

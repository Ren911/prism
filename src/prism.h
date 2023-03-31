#ifndef PRISM_H
#define PRISM_H

#include "polygon3d.h"

using Plane3d = Eigen::Hyperplane<double, 3>;
class Prism {
public:
    //creates prism with two parallel polygons: {first_base} and {first_base + shift}
    Prism(const Polygon3d& first_base, const Eigen::Vector3d& shift);
    //Returns center of mass of prism
    Eigen::Vector3d center() const;
    double getDistanceToPlane(const Plane3d& plane) const;

private:
    Polygon3d first_base_;
    Eigen::Vector3d shift_;
};

#endif // PRISM_H

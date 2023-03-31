#ifndef POLYGON3D_H
#define POLYGON3D_H

#include "polygon.h"

// plane polygon in 3D
class Polygon3d{
public:
    Polygon3d(const std::vector<Eigen::Vector3d>& points);
    Eigen::Vector3d center() const;
private:
    std::vector<Eigen::Vector3d> points_;
    //Returns rotation matrix that transform unit-normal plane
    // of polygon3d to local normal {UnitZ}
    Eigen::Matrix3d getRotationToLocal() const;
    //Transform current polygon3d to flat polygon2d in local coordinates
    //with origin in point0, and such that local z-axis is normal of current polygon
    Polygon getLocalPolygon2d(const Eigen::Matrix3d &rotation_matrix,
                              const Eigen::Vector3d &translation_global) const;
};

#endif // POLYGON3D_H

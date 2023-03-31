#include "prism.h"

Prism::Prism(const Polygon3d &first_base, const Eigen::Vector3d &shift)
    : first_base_(first_base), shift_(shift) {}

Eigen::Vector3d Prism::center() const {
  return first_base_.center() + 0.5 * shift_;
}

double Prism::getDistanceToPlane(const Plane3d &plane) const {
  return plane.absDistance(center());
};

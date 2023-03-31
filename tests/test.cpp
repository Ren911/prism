#define _USE_MATH_DEFINES

#include "math_funcs.h"
#include "polygon.h"
#include "polygon3d.h"
#include "prism.h"
#include "triangle.h"
#include <cmath>
#include <gtest/gtest.h>

bool MatrixEquality(const Eigen::MatrixXd &lhs, const Eigen::MatrixXd &rhs) {
  return lhs.isApprox(rhs, 1e-4);
}

TEST(GeometryFormulasTest, TriagleOrientedArea) {
  // Right triangle
  Eigen::Vector2d p0 = {0.0, 0.0};
  Eigen::Vector2d p1 = {2.0, 0.0};
  Eigen::Vector2d p2 = {0.0, 3.0};

  Triangle triangle012{p0, p1, p2};
  Triangle triangle120{p1, p2, p0};
  Triangle triangle201{p2, p0, p1};

  Triangle triangle102{p1, p0, p2};
  Triangle triangle002{p0, p0, p2};

  EXPECT_DOUBLE_EQ(triangle012.orientedArea(), 3.0);
  EXPECT_DOUBLE_EQ(triangle012.orientedArea(), triangle120.orientedArea());
  EXPECT_DOUBLE_EQ(triangle201.orientedArea(), triangle120.orientedArea());
  EXPECT_DOUBLE_EQ(triangle201.orientedArea(), -triangle102.orientedArea());

  EXPECT_DOUBLE_EQ(triangle002.orientedArea(), 0.0);
}

TEST(GeometryFormulasTest, TriagleCenter) {
  // Equilateral triangle
  double a = 1.0;
  double h = a * sqrt(3.0) / 2.0;
  Eigen::Vector2d p0 = {0.0, 0.0};
  Eigen::Vector2d p1 = {a, 0.0};
  Eigen::Vector2d p2 = {0.5 * a, h};

  Triangle triangle012{p0, p1, p2};
  Triangle triangle102{p1, p0, p2};

  Eigen::Vector2d center_expected = {0.5 * a, h / 3.0};
  ASSERT_PRED2(MatrixEquality, triangle012.center(), center_expected);
  ASSERT_PRED2(MatrixEquality, triangle012.center(), triangle102.center());
}

TEST(GeometryFormulasTest, PolygonSquareCenter) {
  // Square
  Eigen::Vector2d p0 = {0.0, 0.0};
  Eigen::Vector2d p1 = {0.0, 1.0};
  Eigen::Vector2d p2 = {1.0, 1.0};
  Eigen::Vector2d p3 = {1.0, 0.0};

  Polygon polygon0123{{p0, p1, p2, p3}};
  Eigen::Vector2d center_expected = {0.5, 0.5};
  ASSERT_PRED2(MatrixEquality, polygon0123.center(), center_expected);

  Polygon polygon1230{{p1, p2, p3, p0}};
  ASSERT_PRED2(MatrixEquality, polygon0123.center(), polygon1230.center());

  Triangle triangle012{p0, p1, p2};
  Polygon polygon012{{p0, p1, p2}};
  ASSERT_PRED2(MatrixEquality, polygon012.center(), triangle012.center());
}
// generate right n-polygon with origin in center
//  if use_star_shape = true than polygon is non-convex symmetric star
Polygon generateSymmetricPolygon(int n, bool use_star_shape,
                                 Eigen::Vector2d center) {
  std::vector<Eigen::Vector2d> points(n);
  for (int i = 0; i < n; i++) {
    double angle = i * (2.0 * M_PI / n);
    double coeff = 1.0;
    if (use_star_shape && i % 2 == 0) {
      double coeff = 0.5;
    }
    points[i] = {coeff * cos(angle), coeff * sin(angle)};
    points[i] += center;
  }
  return Polygon{points};
}

TEST(GeometryFormulasTest, PolygonRightNCenter) {
  Eigen::Vector2d center_expected = {0.5, 0.5};
  Polygon right_polygon_10 =
      generateSymmetricPolygon(10, false, center_expected);
  ASSERT_PRED2(MatrixEquality, right_polygon_10.center(), center_expected);
  Polygon right_polygon_11 =
      generateSymmetricPolygon(11, false, center_expected);
  ASSERT_PRED2(MatrixEquality, right_polygon_10.center(), center_expected);
}

TEST(GeometryFormulasTest, PolygonNonConvexStarCenter) {
  Eigen::Vector2d center_expected = {0.5, 0.5};
  Polygon star_polygon_10 = generateSymmetricPolygon(10, true, center_expected);
  ASSERT_PRED2(MatrixEquality, star_polygon_10.center(), center_expected);
  Polygon star_polygon_11 = generateSymmetricPolygon(11, true, center_expected);
  ASSERT_PRED2(MatrixEquality, star_polygon_11.center(), center_expected);
}

TEST(GeometryTransformTest, transformVecGlobalLocal) {
  Eigen::Vector3d vec_global = {1.0, 2.0, 3.0};
  vec_global.normalize();
  Eigen::Vector3d vec_local = {-1.0, 2.0, 1};
  vec_local.normalize();
  Eigen::Matrix3d R = math::getGlobalToLocalRotation(vec_global, vec_local);
  ASSERT_PRED2(MatrixEquality, vec_local, R * vec_global);
  ASSERT_PRED2(MatrixEquality, vec_global, R.transpose() * vec_local);
}

TEST(GeometryTransformTest, transformPointGlobalLocal) {
  Eigen::Vector3d point_global = {2.0, 1.0, 3.0};
  Eigen::Matrix3d R{Eigen::AngleAxis(0.5 * M_PI, Eigen::Vector3d::UnitZ())};
  Eigen::Vector3d point_local_expected = {-1, 2, 3.0};
  Eigen::Vector3d reference_point = Eigen::Vector3d::Zero();
  Eigen::Vector3d point_local =
      math::transformToLocal(point_global, R, reference_point);
  ASSERT_PRED2(MatrixEquality, point_local, point_local_expected);
  ASSERT_PRED2(MatrixEquality,
               math::transformToGlobal(point_local, R, reference_point),
               point_global);

  // change reference point with shift
  Eigen::Vector3d shift = {1.0, 2.0, 3.0};
  reference_point += shift;
  point_global += shift;
  point_local = math::transformToLocal(point_global, R, reference_point);
  ASSERT_PRED2(MatrixEquality, point_local, point_local_expected);
  ASSERT_PRED2(MatrixEquality,
               math::transformToGlobal(point_local, R, reference_point),
               point_global);
}

TEST(GeometryFormulasTest, Polygon3dSquareCenter) {
  // Square
  Eigen::Vector2d p0 = {0.0, 0.0};
  Eigen::Vector2d p1 = {0.0, 1.0};
  Eigen::Vector2d p2 = {1.0, 1.0};
  Eigen::Vector2d p3 = {1.0, 0.0};
  std::vector<Eigen::Vector2d> points2d = {p0, p1, p2, p3};
  std::vector<Eigen::Vector3d> points3d;
  points3d.reserve(points2d.size());
  std::transform(points2d.begin(), points2d.end(), std::back_inserter(points3d),
                 [&](const auto &point) { return math::to3D(point); });
  Polygon polygon{points2d};
  Polygon3d polygon3d{points3d};
  ASSERT_PRED2(MatrixEquality, polygon3d.center(),
               math::to3D(polygon.center()));
  ASSERT_PRED2(MatrixEquality, polygon.center(),
               math::to2D(polygon3d.center()));

  // rotate polygon3d and shift
  std::vector<Eigen::Vector3d> points3d_transformed;
  points3d_transformed.reserve(points3d.size());
  Eigen::Vector3d axis = {1.0, 1.0, 1.0};
  axis.normalize();
  Eigen::Matrix3d R{Eigen::AngleAxis(M_PI / 3.0, axis)};
  Eigen::Vector3d shift = {2.0, 10.0, -1.0};
  auto affine_transform = [&](const Eigen::Vector3d &point) {
    return R * point + shift;
  };

  std::transform(points3d.begin(), points3d.end(),
                 std::back_inserter(points3d_transformed), affine_transform);
  Polygon3d polygon3d_transformed{points3d_transformed};
  ASSERT_PRED2(MatrixEquality, polygon3d_transformed.center(),
               affine_transform(polygon3d.center()));
}

TEST(GeometryFormulasTest, PrismParallepipedCenter) {
  Eigen::Vector3d p0 = {0.0, 0.0, 0.0};
  Eigen::Vector3d p1 = {0.0, 1.0, 0.0};
  Eigen::Vector3d p2 = {1.0, 1.0, 0.0};
  Eigen::Vector3d p3 = {1.0, 0.0, 0.0};
  std::vector<Eigen::Vector3d> points3d = {p0, p1, p2, p3};
  Eigen::Vector3d shift_base = {0.0, 0.0, 1.0};
  Prism prism{points3d, shift_base};
  Eigen::Vector3d center_expected = {0.5, 0.5, 0.5};
  ASSERT_PRED2(MatrixEquality, prism.center(), center_expected);

  Plane3d planeZ = {Eigen::Vector3d::UnitZ(), 2 * Eigen::Vector3d::UnitZ()};
  EXPECT_DOUBLE_EQ(prism.getDistanceToPlane(planeZ), 1.5);

  Plane3d planeY = {Eigen::Vector3d::UnitY(), 2 * Eigen::Vector3d::UnitY()};
  EXPECT_DOUBLE_EQ(prism.getDistanceToPlane(planeZ), 1.5);

  Plane3d planeX = {Eigen::Vector3d::UnitX(), 2 * Eigen::Vector3d::UnitX()};
  EXPECT_DOUBLE_EQ(prism.getDistanceToPlane(planeX), 1.5);

  // rotate polygon3d and shift
  std::vector<Eigen::Vector3d> points3d_transformed;
  points3d_transformed.reserve(points3d.size());
  Eigen::Vector3d axisXYZ = {1.0, 1.0, 1.0};
  axisXYZ.normalize();
  Eigen::Matrix3d R{Eigen::AngleAxis(M_PI / 3.0, axisXYZ)};
  Eigen::Vector3d shift = {2.0, 10.0, -1.0};
  auto affine_transform = [&](const Eigen::Vector3d &point) {
    return R * point + shift;
  };
  Eigen::Vector3d shift_base_transformed = R * (shift_base); // as vector
  std::transform(points3d.begin(), points3d.end(),
                 std::back_inserter(points3d_transformed), affine_transform);
  Polygon3d polygon3d_transformed{points3d_transformed};
  Prism prism_transformed{points3d_transformed, shift_base_transformed};
  ASSERT_PRED2(MatrixEquality, prism_transformed.center(),
               affine_transform(center_expected));
  Eigen::Vector3d plane_normal = Eigen::Vector3d{1.0, 1.0, 1.0}.normalized();
  Eigen::Vector3d plane_point = Eigen::Vector3d{1.0, 2.0, -3.0};
  Plane3d random_plane = {plane_normal, plane_point};
  Plane3d random_plane_transformed = {R * plane_normal,
                                      affine_transform(plane_point)};
  EXPECT_DOUBLE_EQ(
      prism.getDistanceToPlane(random_plane),
      prism_transformed.getDistanceToPlane(random_plane_transformed));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

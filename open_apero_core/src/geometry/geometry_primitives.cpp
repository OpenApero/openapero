#include "open_apero_core/geometry/geometry_primitives.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "open_apero_core/common/typedef.h"

namespace open_apero {

bool isInsideTriangle2D(const Eigen::Vector2d& P, const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) {
  // Barycentric coordinates of P with respect to triangle ABC avoid orientation-dependent edge tests.
  double denominator = ((B(1) - C(1)) * (A(0) - C(0)) + (C(0) - B(0)) * (A(1) - C(1)));

  double a = ((B(1) - C(1)) * (P(0) - C(0)) + (C(0) - B(0)) * (P(1) - C(1))) / denominator;
  double b = ((C(1) - A(1)) * (P(0) - C(0)) + (A(0) - C(0)) * (P(1) - C(1))) / denominator;
  double c = 1 - a - b;

  // Check if all barycentric coordinates
  // are non-negative
  if (a >= 0 && b >= 0 && c >= 0) {
    return true;
  } else {
    return false;
  }
}

bool isInsideTriangularPrism(const Eigen::Vector3d& P, const TriangularPrism& triangle) {
  const Eigen::Vector3d& A = triangle.vertices[0];
  const Eigen::Vector3d& B = triangle.vertices[1];
  const Eigen::Vector3d& C = triangle.vertices[2];
  if (std::abs(A.z() - B.z()) > kEpsilon || std::abs(A.z() - C.z()) > kEpsilon) {
    throw std::runtime_error("TriangularPrism base vertices must share the same Z value");
  }
  const double base_z = A.z();
  if (P.z() < base_z || P.z() > base_z + triangle.height) {
    return false;
  }
  return isInsideTriangle2D(P.head<2>(), A.head<2>(), B.head<2>(), C.head<2>());
}

bool isInsideCylinder(const Eigen::Vector3d& P, const Cylinder& cylinder) {
  const Eigen::Vector3d& bottom_center = cylinder.bottom_center;
  const double radius = cylinder.radius;
  const double height = cylinder.height;
  if (P.z() < bottom_center.z() || P.z() > bottom_center.z() + height) {
    return false;
  }
  return (P.head<2>() - bottom_center.head<2>()).squaredNorm() < radius * radius;
}

void GeometryPrimitiveCollection::clear() {
  spheres.clear();
  cylinders.clear();
  triangles.clear();
}

void GeometryPrimitiveCollection::append(const GeometryPrimitiveCollection& other) {
  spheres.insert(spheres.end(), other.spheres.begin(), other.spheres.end());
  cylinders.insert(cylinders.end(), other.cylinders.begin(), other.cylinders.end());
  triangles.insert(triangles.end(), other.triangles.begin(), other.triangles.end());
}

bool GeometryPrimitiveCollection::getBounds(Eigen::Vector3d* min, Eigen::Vector3d* max) const {
  if (spheres.empty() && cylinders.empty() && triangles.empty()) {
    return false;
  }

  Eigen::Vector3d lower = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d upper = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  const auto extend = [&lower, &upper](const Eigen::Vector3d& point) {
    lower = lower.cwiseMin(point);
    upper = upper.cwiseMax(point);
  };
  for (const Sphere& sphere : spheres) {
    extend(sphere.first - Eigen::Vector3d::Constant(sphere.second));
    extend(sphere.first + Eigen::Vector3d::Constant(sphere.second));
  }
  for (const Cylinder& cylinder : cylinders) {
    extend(cylinder.bottom_center - Eigen::Vector3d(cylinder.radius, cylinder.radius, 0.0));
    extend(cylinder.bottom_center + Eigen::Vector3d(cylinder.radius, cylinder.radius, cylinder.height));
  }
  for (const TriangularPrism& prism : triangles) {
    for (const Eigen::Vector3d& vertex : prism.vertices) {
      extend(vertex);
      extend(vertex + Eigen::Vector3d(0.0, 0.0, prism.height));
    }
  }

  *min = lower;
  *max = upper;
  return true;
}

}  // namespace open_apero

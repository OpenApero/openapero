#pragma once

/**
 * @file geometry_primitives.h
 * @brief Geometric primitive types and geometry tests.
 *
 * Defines sphere, cylinder, and triangular prism primitives along with point-in-triangle
 * tests and a collection structure for storing primitive data.
 */

#include <array>
#include <utility>
#include <vector>

#include <Eigen/Core>

namespace open_apero {

/// @brief A sphere defined as (center, radius).
using Sphere = std::pair<Eigen::Vector3d, double>;

/// @brief A cylinder defined by its bottom center, radius, and height.
struct Cylinder {
  Eigen::Vector3d bottom_center;
  double radius;
  double height;
};

/// @brief A triangular prism defined by three base vertices and a height along Z.
struct TriangularPrism {
  std::array<Eigen::Vector3d, 3> vertices;
  double height;
};

/**
 * @brief Test whether a 2D point lies inside a triangle defined by three 2D vertices.
 * @param P The point to test.
 * @param A First vertex of the triangle.
 * @param B Second vertex of the triangle.
 * @param C Third vertex of the triangle.
 * @return True if the point is inside the triangle.
 */
bool isInsideTriangle2D(const Eigen::Vector2d& P, const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C);

/**
 * @brief Test whether a 3D point lies inside a triangular prism.
 *        The prism spans [base_z, base_z + height] along Z.
 * @param P The point to test.
 * @param triangle Triangular prism with coplanar base vertices and a height.
 * @return True if the point is inside the prism.
 */
bool isInsideTriangularPrism(const Eigen::Vector3d& P, const TriangularPrism& triangle);

/**
 * @brief Test whether a 3D point lies inside a Z-axis-aligned cylinder.
 *        The cylinder spans [bottom_center.z, bottom_center.z + height].
 * @param P The point to test.
 * @param cylinder The cylinder (bottom_center, radius, height).
 * @return True if the point is inside the cylinder.
 */
bool isInsideCylinder(const Eigen::Vector3d& P, const Cylinder& cylinder);

/**
 * @brief A collection of geometry primitives used to represent hit/blocked regions.
 */
struct GeometryPrimitiveCollection {
  /// @brief Collection of sphere primitives.
  std::vector<Sphere> spheres;

  /// @brief Collection of cylinder primitives.
  std::vector<Cylinder> cylinders;

  /// @brief Collection of triangular prism primitives (named for their triangular bases).
  std::vector<TriangularPrism> triangles;

  /**
   * @brief Clear all primitives from the collection.
   */
  void clear();

  /**
   * @brief Append all primitives from another collection into this one.
   * @param other The collection to append.
   */
  void append(const GeometryPrimitiveCollection& other);

  /**
   * @brief Axis-aligned bounds of every primitive in the collection.
   * @param min Output minimum corner.
   * @param max Output maximum corner.
   * @return false, with @p min and @p max unchanged, when the collection is empty.
   */
  bool getBounds(Eigen::Vector3d* min, Eigen::Vector3d* max) const;
};

}  // namespace open_apero

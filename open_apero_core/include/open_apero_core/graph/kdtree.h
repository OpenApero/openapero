#pragma once

/**
 * @file kdtree.h
 * @brief KD-tree point clouds over 3D positions, and their radius query.
 *
 * Every point cloud derives from KdTreePoints, the nanoflann point-cloud interface over its
 * positions, and keeps its per-point payload parallel to KdTreePoints::points. KdTreeData's
 * payload is the cluster of planning nodes at each point, ViewpointKdTreeData's the sensor
 * orientation of the viewpoint at each point.
 *
 * @see https://github.com/jlblancoc/nanoflann/blob/master/examples/utils.h
 */

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "open_apero_core/common/typedef.h"
#include "open_apero_core/graph/node.h"
#include "open_apero_core/third_party/nanoflann.hpp"

namespace open_apero {

/**
 * @brief The nanoflann point-cloud interface over a list of 3D positions.
 */
struct KdTreePoints {
  /// @brief Stored 3D point positions; a KD-tree point's index is its index here.
  std::vector<Eigen::Vector3d> points;

  /**
   * @brief Return the number of points for nanoflann KD-tree interface.
   * @return Number of stored points.
   */
  inline std::size_t kdtree_get_point_count() const { return points.size(); }

  /**
   * @brief Get a coordinate component of a point for nanoflann KD-tree interface.
   * @param idx Index of the point.
   * @param dim Dimension (0=x, 1=y, 2=z).
   * @return The coordinate value.
   */
  inline double kdtree_get_pt(const std::size_t idx, const std::size_t dim) const {
    if (dim == 0)
      return points[idx].x();
    else if (dim == 1)
      return points[idx].y();
    else
      return points[idx].z();
  }

  /**
   * @brief Optional bounding box computation for nanoflann (not implemented).
   * @tparam BBOX Bounding box type.
   * @return Always returns false.
   */
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
    return false;
  }
};

/**
 * @brief Point cloud data adapter for nanoflann KD-tree, mapping 3D points to node clusters.
 *
 * Stores 3D points alongside associated node clusters. Points that are close
 * together (within a squared epsilon) are merged into the same cluster.
 * Intermediate nodes are not stored.
 */
struct KdTreeData : KdTreePoints {
  /// @brief Node clusters associated with each point.
  std::vector<NodeClusterPtrs> nodes;

  /**
   * @brief Clear all points and node clusters.
   */
  inline void clear() {
    points.clear();
    nodes.clear();
  }

  /**
   * @brief Get the number of stored points.
   * @return Number of points.
   */
  inline int size() const { return points.size(); }

  /**
   * @brief Add a node at a given position, merging into existing clusters if nearby.
   *
   * Intermediate nodes are rejected (returns -1). If the point is within a
   * squared epsilon of an existing point, the node is appended to that cluster.
   *
   * @param point The 3D position of the node.
   * @param node Pointer to the node to add.
   * @return The index of the point in the data, or -1 if the node is intermediate.
   */
  inline int addNode(const Eigen::Vector3d& point, Node* node) {
    // Intermediate nodes are transient execution cursors, not search samples.
    if (node->is_intermediate) {
      return -1;
    }

    // Co-located orientations share one KD-tree point.
    constexpr double kSquaredEps = 1e-8;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
      if ((points[i] - point).squaredNorm() < kSquaredEps) {
        nodes[i].emplace_back(node);
        return i;
      }
    }

    points.emplace_back(point);
    nodes.emplace_back(std::vector<Node*>{node});
    return points.size() - 1;
  }
};

/// @brief Dynamic 3D KD-tree type using L2 distance over KdTreeData.
using KdTree = nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, KdTreeData>, KdTreeData, 3>;

/**
 * @brief Point cloud of sensor viewpoints: positions indexed, orientations carried alongside.
 *
 * Holds bare poses, which no graph node stands for; a pose's index is its index in both vectors.
 */
struct ViewpointKdTreeData : KdTreePoints {
  /// @brief Sensor orientation of each viewpoint, parallel to KdTreePoints::points.
  std::vector<Eigen::Quaterniond> orientations;
};

/// @brief Dynamic 3D KD-tree over viewpoint positions, using L2 distance.
using ViewpointKdTree =
    nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<double, ViewpointKdTreeData>, ViewpointKdTreeData, 3>;

/**
 * @brief Indices of the points of a KD-tree closer than a radius to a position.
 *
 * @tparam Tree A nanoflann KD-tree over a KdTreePoints point cloud, with L2 distance.
 * @param tree The KD-tree to search.
 * @param position Query position.
 * @param radius Search radius [m]; infinity returns every point.
 * @return The point indices, in KD-tree traversal order (unsorted).
 */
template <class Tree>
std::vector<std::size_t> findPointsWithin(const Tree& tree, const Eigen::Vector3d& position, double radius) {
  const double query_point[3] = {position.x(), position.y(), position.z()};
  std::vector<nanoflann::ResultItem<std::size_t, double>> indices_dists;
  nanoflann::RadiusResultSet<double, std::size_t> result_set(radius * radius, indices_dists);
  tree.findNeighbors(result_set, query_point);

  std::vector<std::size_t> indices;
  indices.reserve(indices_dists.size());
  for (const auto& item : indices_dists) {
    indices.push_back(item.first);
  }
  return indices;
}

}  // namespace open_apero

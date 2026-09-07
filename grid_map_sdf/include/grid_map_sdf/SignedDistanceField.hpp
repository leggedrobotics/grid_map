/*
 * SignedDistanceField.hpp
 *
 *  Created on: Aug 16, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_SDF__SIGNEDDISTANCEFIELD_HPP_
#define GRID_MAP_SDF__SIGNEDDISTANCEFIELD_HPP_

#pragma once

#include <grid_map_core/GridMap.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <string>

namespace grid_map
{

/** Exact signed distance to occupied height-map cell volumes.
 *
 * A finite elevation fills its complete GridMap cell footprint downwards.
 * Nonfinite cells and the map exterior are occupied at every height. Queries
 * themselves must lie over a finite cell; their clearance includes unknown and
 * exterior boundaries. Values are continuous exact geometric queries, including
 * between grid samples, rather than interpolation of sampled distances.
 *
 * The distance is differentiable except at geometric edges and equal-nearest
 * features. At such points one deterministic outward derivative is returned.
 * Queries at any finite z are supported; heightClearance affects only
 * point-cloud export. Build once, then query concurrently without modifying
 * this object.
 */
class SignedDistanceField {
public:
  struct DistanceAndGradient
  {
    double distance;
    Vector3 gradient;
  };

  struct QueryStatistics
  {
    std::size_t nodesVisited = 0;
    std::size_t facesTested = 0;
  };

  SignedDistanceField();
  virtual ~SignedDistanceField();

  /// Rejects empty/all-unknown maps or invalid geometry/settings.
  /// Preserves a previous build on failure.
  void calculateSignedDistanceField(
    const GridMap & gridMap,
    const std::string & layer,
    const double heightClearance);

  /// True for finite positions over known cell coverage (including the outer
  /// cell edges).
  bool isInside(const Position3 & position) const;

  /// Throws on unbuilt fields, nonfinite positions, unknown cells, or
  /// outside-map queries.
  DistanceAndGradient
  getDistanceAndGradientAt(
    const Position3 & position,
    QueryStatistics *statistics = nullptr) const;

  /// Horizontal distance to unknown cells or the map exterior, independent of
  /// terrain height. Contact exemptions must still keep body volumes in
  /// coverage.
  DistanceAndGradient
  getKnownCoverageDistanceAndGradientAt(const Position3 & position) const;

  double getDistanceAt(const Position3 & position) const;
  Vector3 getDistanceGradientAt(const Position3 & position) const;

  /// Retained API: evaluates the same exact continuous distance as
  /// getDistanceAt().
  double getInterpolatedDistanceAt(const Position3 & position) const;

  /// Appends samples at known GridMap cell centres from min height through max
  /// height + clearance.
  void convertToPointCloud(pcl::PointCloud<pcl::PointXYZI> & points) const;

private:
  struct Data;
  std::shared_ptr<const Data> data_;
};

}  // namespace grid_map

#endif  // GRID_MAP_SDF__SIGNEDDISTANCEFIELD_HPP_

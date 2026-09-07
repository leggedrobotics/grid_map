/*
 * SignedDistanceField.cpp
 *
 *  Created on: Aug 16, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_sdf/SignedDistanceField.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace grid_map
{
namespace
{

Vector3 projectToBox(
  const Vector3 & point, const Vector3 & minimum,
  const Vector3 & maximum)
{
  return point.cwiseMax(minimum).cwiseMin(maximum);
}

double squaredDistanceToBox(
  const Vector3 & point, const Vector3 & minimum,
  const Vector3 & maximum)
{
  return (point - projectToBox(point, minimum, maximum)).squaredNorm();
}

}  // namespace

struct SignedDistanceField::Data
{
  struct Face
  {
    Vector3 minimum;
    Vector3 maximum;
    Vector3 normal;
    std::size_t order;

    Vector3 centre() const
    {
      Vector3 result = 0.5 * (minimum + maximum);
      // Unknown/exterior walls extend to +infinity. Their finite lower endpoint
      // supplies a sorting coordinate; their query bounds remain unbounded.
      if (!std::isfinite(result.z())) {
        result.z() = minimum.z();
      }
      return result;
    }
  };

  struct Node
  {
    Vector3 minimum;
    Vector3 maximum;
    std::size_t begin;
    std::size_t end;
    std::size_t left = 0;
    std::size_t right = 0;
  };

  struct Nearest
  {
    double squaredDistance;
    Vector3 point;
    Vector3 normal;
    std::size_t order = std::numeric_limits<std::size_t>::max();
  };

  Matrix elevations;
  Position minimum;
  Position maximum;
  double resolution;
  double minHeight;
  double maxHeight;
  double heightClearance;
  std::vector<Face> faces;
  std::vector<Node> nodes;

  double height(int row, int column) const
  {
    if (row < 0 || column < 0 || row >= elevations.rows() ||
      column >= elevations.cols())
    {
      return std::numeric_limits<double>::infinity();
    }
    const double value = elevations(row, column);
    return std::isfinite(value) ? value :
           std::numeric_limits<double>::infinity();
  }

  bool index(const Position3 & point, Index & result) const
  {
    if (!point.allFinite() || point.x() < minimum.x() ||
      point.x() > maximum.x() || point.y() < minimum.y() ||
      point.y() > maximum.y())
    {
      return false;
    }
    result.x() =
      std::min(static_cast<int>((maximum.x() - point.x()) / resolution),
                 static_cast<int>(elevations.rows()) - 1);
    result.y() =
      std::min(static_cast<int>((maximum.y() - point.y()) / resolution),
                 static_cast<int>(elevations.cols()) - 1);
    return std::isfinite(elevations(result.x(), result.y()));
  }

  void addFace(
    const Vector3 & lower, const Vector3 & upper,
    const Vector3 & normal)
  {
    faces.push_back({lower, upper, normal, faces.size()});
  }

  void makeFaces()
  {
    const int rows = static_cast<int>(elevations.rows());
    const int columns = static_cast<int>(elevations.cols());
    // Merge equal-height runs, including uniform unknown regions. Flat ground
    // does not need one primitive per cell. No geometric approximation is used.
    for (int row = 0; row < rows; ++row) {
      for (int column = 0; column < columns; ) {
        const double z = height(row, column);
        int end = column + 1;
        while (end < columns && height(row, end) == z) {
          ++end;
        }
        if (std::isfinite(z)) {
          addFace(Vector3(maximum.x() - (row + 1) * resolution,
                          maximum.y() - end * resolution, z),
                  Vector3(maximum.x() - row * resolution,
                          maximum.y() - column * resolution, z),
                  Vector3::UnitZ());
        }
        column = end;
      }
    }

    // Every shared edge is visited once. Only the exposed interval between the
    // adjacent heights belongs to the volume boundary; internal buried faces do
    // not.
    for (int row = 0; row <= rows; ++row) {
      for (int column = 0; column < columns; ) {
        const double highSide = height(row - 1, column);
        const double lowSide = height(row, column);
        int end = column + 1;
        while (end < columns && height(row - 1, end) == highSide &&
          height(row, end) == lowSide)
        {
          ++end;
        }
        if (highSide != lowSide) {
          const double x = maximum.x() - row * resolution;
          addFace(Vector3(x, maximum.y() - end * resolution,
                          std::min(highSide, lowSide)),
                  Vector3(x, maximum.y() - column * resolution,
                          std::max(highSide, lowSide)),
                  highSide > lowSide ? Vector3(-1.0, 0.0, 0.0) :
                                       Vector3::UnitX());
        }
        column = end;
      }
    }
    for (int column = 0; column <= columns; ++column) {
      for (int row = 0; row < rows; ) {
        const double highSide = height(row, column - 1);
        const double lowSide = height(row, column);
        int end = row + 1;
        while (end < rows && height(end, column - 1) == highSide &&
          height(end, column) == lowSide)
        {
          ++end;
        }
        if (highSide != lowSide) {
          const double y = maximum.y() - column * resolution;
          addFace(Vector3(maximum.x() - end * resolution, y,
                          std::min(highSide, lowSide)),
                  Vector3(maximum.x() - row * resolution, y,
                          std::max(highSide, lowSide)),
                  highSide > lowSide ? Vector3(0.0, -1.0, 0.0) :
                                       Vector3::UnitY());
        }
        row = end;
      }
    }
  }

  std::size_t buildNode(std::size_t begin, std::size_t end)
  {
    const double infinity = std::numeric_limits<double>::infinity();
    Node node{Vector3::Constant(infinity), Vector3::Constant(-infinity), begin,
      end};
    Vector3 centreMinimum = node.minimum;
    Vector3 centreMaximum = node.maximum;
    for (std::size_t i = begin; i < end; ++i) {
      node.minimum = node.minimum.cwiseMin(faces[i].minimum);
      node.maximum = node.maximum.cwiseMax(faces[i].maximum);
      const Vector3 centre = faces[i].centre();
      centreMinimum = centreMinimum.cwiseMin(centre);
      centreMaximum = centreMaximum.cwiseMax(centre);
    }
    const std::size_t index = nodes.size();
    nodes.push_back(node);
    constexpr std::size_t leafSize = 8;
    if (end - begin > leafSize) {
      Eigen::Index axis;
      (centreMaximum - centreMinimum).maxCoeff(&axis);
      const std::size_t middle = begin + (end - begin) / 2;
      std::nth_element(faces.begin() + begin, faces.begin() + middle,
                       faces.begin() + end,
        [axis](const Face & lhs, const Face & rhs) {
          const double a = lhs.centre()(axis);
          const double b = rhs.centre()(axis);
          return a == b ? lhs.order < rhs.order : a < b;
                       });
      const std::size_t left = buildNode(begin, middle);
      const std::size_t right = buildNode(middle, end);
      nodes[index].left = left;
      nodes[index].right = right;
    }
    return index;
  }

  void findNearest(
    std::size_t index, const Position3 & point, Nearest & nearest,
    QueryStatistics *statistics) const
  {
    const Node & node = nodes[index];
    if (statistics != nullptr) {
      ++statistics->nodesVisited;
    }
    if (squaredDistanceToBox(point, node.minimum, node.maximum) >
      nearest.squaredDistance)
    {
      return;
    }
    if (node.left == 0) {
      for (std::size_t i = node.begin; i < node.end; ++i) {
        const Face & face = faces[i];
        const Vector3 projection =
          projectToBox(point, face.minimum, face.maximum);
        const double squaredDistance = (point - projection).squaredNorm();
        if (statistics != nullptr) {
          ++statistics->facesTested;
        }
        if (squaredDistance < nearest.squaredDistance ||
          (squaredDistance == nearest.squaredDistance &&
          face.order < nearest.order))
        {
          nearest = {squaredDistance, projection, face.normal, face.order};
        }
      }
      return;
    }
    const Node & left = nodes[node.left];
    const Node & right = nodes[node.right];
    const double leftDistance =
      squaredDistanceToBox(point, left.minimum, left.maximum);
    const double rightDistance =
      squaredDistanceToBox(point, right.minimum, right.maximum);
    if (leftDistance <= rightDistance) {
      findNearest(node.left, point, nearest, statistics);
      findNearest(node.right, point, nearest, statistics);
    } else {
      findNearest(node.right, point, nearest, statistics);
      findNearest(node.left, point, nearest, statistics);
    }
  }
};

SignedDistanceField::SignedDistanceField() = default;
SignedDistanceField::~SignedDistanceField() = default;

void SignedDistanceField::calculateSignedDistanceField(
  const GridMap & gridMap, const std::string & layer,
  const double heightClearance)
{
  if (!gridMap.exists(layer) || !std::isfinite(heightClearance) ||
    heightClearance < 0.0 || !std::isfinite(gridMap.getResolution()) ||
    gridMap.getResolution() <= 0.0 || !gridMap.getPosition().allFinite() ||
    !gridMap.getLength().allFinite() || (gridMap.getSize() <= 0).any())
  {
    throw std::invalid_argument("SignedDistanceField requires a nonempty map, "
                                "layer, and finite valid geometry/settings.");
  }

  // Copy only the selected layer, preserving then normalizing circular-buffer
  // indices.
  GridMap normalized(std::vector<std::string>{layer});
  normalized.setGeometry(gridMap.getLength(), gridMap.getResolution(),
                         gridMap.getPosition());
  normalized[layer] = gridMap.get(layer);
  normalized.setStartIndex(gridMap.getStartIndex());
  normalized.convertToDefaultStartIndex();

  auto next = std::make_shared<Data>();
  next->elevations = std::move(normalized[layer]);
  next->resolution = gridMap.getResolution();
  next->minimum = gridMap.getPosition() - 0.5 * gridMap.getLength().matrix();
  next->maximum = gridMap.getPosition() + 0.5 * gridMap.getLength().matrix();
  next->heightClearance = heightClearance;
  next->minHeight = std::numeric_limits<double>::infinity();
  next->maxHeight = -std::numeric_limits<double>::infinity();
  for (Eigen::Index i = 0; i < next->elevations.size(); ++i) {
    const double value = next->elevations(i);
    if (std::isfinite(value)) {
      next->minHeight = std::min(next->minHeight, value);
      next->maxHeight = std::max(next->maxHeight, value);
    }
  }
  if (!std::isfinite(next->minHeight) || !next->minimum.allFinite() ||
    !next->maximum.allFinite())
  {
    throw std::invalid_argument("SignedDistanceField requires at least one "
                                "finite elevation and finite map bounds.");
  }
  next->makeFaces();
  next->nodes.reserve(2 * next->faces.size());
  next->buildNode(0, next->faces.size());
  data_ = std::move(next);
}

bool SignedDistanceField::isInside(const Position3 & position) const
{
  Index index;
  return data_ != nullptr && data_->index(position, index);
}

SignedDistanceField::DistanceAndGradient
SignedDistanceField::getDistanceAndGradientAt(
  const Position3 & position, QueryStatistics *statistics) const
{
  if (statistics != nullptr) {
    *statistics = QueryStatistics{};
  }
  if (data_ == nullptr) {
    throw std::logic_error("SignedDistanceField has not been constructed.");
  }
  if (!position.allFinite()) {
    throw std::invalid_argument(
        "SignedDistanceField query position must be finite.");
  }
  Index index;
  if (!data_->index(position, index)) {
    throw std::out_of_range(
        "SignedDistanceField query is outside known map-cell coverage.");
  }
  const double height = data_->elevations(index.x(), index.y());
  const Vector3 top(position.x(), position.y(), height);
  Data::Nearest nearest{(position - top).squaredNorm(), top, Vector3::UnitZ()};
  data_->findNearest(0, position, nearest, statistics);
  const double magnitude = std::sqrt(nearest.squaredDistance);
  if (!std::isfinite(magnitude)) {
    throw std::overflow_error(
        "SignedDistanceField distance is not representable.");
  }
  const double sign = position.z() < height ? -1.0 : 1.0;
  const Vector3 gradient = magnitude > 0.0 ?
    sign * (position - nearest.point) / magnitude :
    nearest.normal;
  return {sign * magnitude, gradient};
}

SignedDistanceField::DistanceAndGradient
SignedDistanceField::getKnownCoverageDistanceAndGradientAt(
  const Position3 & position) const
{
  if (data_ == nullptr) {
    throw std::logic_error("SignedDistanceField has not been constructed.");
  }
  if (!position.allFinite()) {
    throw std::invalid_argument(
        "SignedDistanceField query position must be finite.");
  }
  // Above every finite face by more than the map diagonal, the nearest
  // boundary is necessarily an unbounded unknown/exterior wall. Querying the
  // existing exact BVH there gives the horizontal coverage distance directly.
  const double separation = (data_->maximum - data_->minimum).norm() + 1.0;
  const double height = std::nextafter(data_->maxHeight + separation,
                                       std::numeric_limits<double>::infinity());
  if (!std::isfinite(height) || height - data_->maxHeight < separation) {
    throw std::overflow_error(
        "SignedDistanceField coverage-query height is not representable.");
  }
  return getDistanceAndGradientAt(
      Position3(position.x(), position.y(), height));
}

double SignedDistanceField::getDistanceAt(const Position3 & position) const
{
  return getDistanceAndGradientAt(position).distance;
}

double SignedDistanceField::getInterpolatedDistanceAt(
  const Position3 & position) const
{
  return getDistanceAt(position);
}

Vector3
SignedDistanceField::getDistanceGradientAt(const Position3 & position) const
{
  return getDistanceAndGradientAt(position).gradient;
}

void SignedDistanceField::convertToPointCloud(
  pcl::PointCloud<pcl::PointXYZI> & points) const
{
  if (data_ == nullptr) {
    throw std::logic_error("SignedDistanceField has not been constructed.");
  }
  const double steps =
    (data_->maxHeight - data_->minHeight + data_->heightClearance) /
    data_->resolution;
  if (!std::isfinite(steps) ||
    steps >= static_cast<double>(std::numeric_limits<std::size_t>::max()))
  {
    throw std::length_error(
        "SignedDistanceField point-cloud height range is too large.");
  }
  const std::size_t slices = static_cast<std::size_t>(std::floor(steps)) + 1;
  for (std::size_t k = 0; k < slices; ++k) {
    const double z = data_->minHeight + k * data_->resolution;
    for (Eigen::Index row = 0; row < data_->elevations.rows(); ++row) {
      for (Eigen::Index column = 0; column < data_->elevations.cols();
        ++column)
      {
        if (!std::isfinite(data_->elevations(row, column))) {
          continue;
        }
        const Vector3 position(
          data_->maximum.x() - (row + 0.5) * data_->resolution,
          data_->maximum.y() - (column + 0.5) * data_->resolution, z);
        pcl::PointXYZI point;
        point.x = static_cast<float>(position.x());
        point.y = static_cast<float>(position.y());
        point.z = static_cast<float>(position.z());
        point.intensity = static_cast<float>(getDistanceAt(position));
        points.push_back(point);
      }
    }
  }
}

}  // namespace grid_map

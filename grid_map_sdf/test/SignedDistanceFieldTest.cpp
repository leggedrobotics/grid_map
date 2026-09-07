/*
 * SignedDistanceFieldTest.cpp
 *
 *  Created on: Aug 25, 2017
 *     Authors: Takahiro Miki, Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/GridMapIterator.hpp"
#include "grid_map_sdf/SignedDistanceField.hpp"

namespace
{
using grid_map::GridMap;
using grid_map::Index;
using grid_map::Length;
using grid_map::Position;
using grid_map::Position3;
using grid_map::SignedDistanceField;
using grid_map::Vector3;

GridMap makeMap(double length = 8.0, double resolution = 0.25)
{
  GridMap map({"elevation"});
  map.setGeometry(Length(length, length), resolution, Position::Zero());
  map["elevation"].setZero();
  return map;
}

template<typename Function> void setHeights(GridMap & map, Function function)
{
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
    ++iterator)
  {
    Position position;
    map.getPosition(*iterator, position);
    map.at("elevation", *iterator) = function(position);
  }
}

SignedDistanceField makeField(const GridMap & map, double clearance = 3.0)
{
  SignedDistanceField field;
  field.calculateSignedDistanceField(map, "elevation", clearance);
  return field;
}

// Independent reference: distance to a UNION OF VOLUMES, not to the
// implementation's exposed-face mesh. In free space, minimize distance to
// downward occupied prisms. Inside terrain, minimize distance to known upward
// free-space prisms and negate.
double referenceDistance(const GridMap & map, const Position3 & point)
{
  Index containing;
  if (!map.getIndex(point.head<2>(), containing)) {
    throw std::out_of_range("reference query outside map");
  }
  const double localHeight = map.at("elevation", containing);
  if (!std::isfinite(localHeight)) {
    throw std::out_of_range("reference query in unknown cell");
  }
  const bool inside = point.z() < localHeight;
  const Position minimum = map.getPosition() - 0.5 * map.getLength().matrix();
  const Position maximum = map.getPosition() + 0.5 * map.getLength().matrix();
  const double halfCell = 0.5 * map.getResolution();
  double distance = std::numeric_limits<double>::infinity();
  if (!inside) {
    distance =
      std::min(std::min(point.x() - minimum.x(), maximum.x() - point.x()),
                 std::min(point.y() - minimum.y(), maximum.y() - point.y()));
  }
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
    ++iterator)
  {
    const double height = map.at("elevation", *iterator);
    if (inside && !std::isfinite(height)) {
      continue;
    }
    Position centre;
    map.getPosition(*iterator, centre);
    const double dx =
      std::max(0.0, std::abs(point.x() - centre.x()) - halfCell);
    const double dy =
      std::max(0.0, std::abs(point.y() - centre.y()) - halfCell);
    const double dz = !std::isfinite(height) ?
      0.0 :
      (inside ? std::max(0.0, height - point.z()) :
      std::max(0.0, point.z() - height));
    distance = std::min(distance, std::sqrt(dx * dx + dy * dy + dz * dz));
  }
  return inside ? -distance : distance;
}

TEST(SignedDistanceField, FlatGroundAndCoverageAreExact) {
  const auto field = makeField(makeMap());
  EXPECT_DOUBLE_EQ(field.getDistanceAt(Position3(0.0, 0.0, -3.0)), -3.0);
  EXPECT_DOUBLE_EQ(field.getDistanceAt(Position3(0.0, 0.0, 0.75)), 0.75);
  EXPECT_DOUBLE_EQ(field.getDistanceAt(Position3(0.0, 0.0, 100.0)), 4.0);
  EXPECT_DOUBLE_EQ(field.getDistanceAt(Position3(3.9, 0.0, 1.0)), 4.0 - 3.9);
  EXPECT_DOUBLE_EQ(field.getDistanceAt(Position3(4.0, 0.0, 1.0)), 0.0);
  EXPECT_DOUBLE_EQ(field.getDistanceAt(Position3(3.9, 0.0, -0.5)), -0.5);
  EXPECT_TRUE(field.getDistanceGradientAt(Position3(0.0, 0.0, 0.75))
    .isApprox(Vector3::UnitZ()));
  EXPECT_TRUE(field.getDistanceGradientAt(Position3(3.9, 0.0, 1.0))
    .isApprox(Vector3(-1.0, 0.0, 0.0)));
  EXPECT_THROW(field.getDistanceAt(Position3(4.00001, 0.0, 1.0)),
               std::out_of_range);
}

TEST(SignedDistanceField, StepRegressionUsesCellEdgesAndEuclideanCorners) {
  auto map = makeMap();
  setHeights(
      map, [](const Position & point) {return point.x() >= 0.0 ? 2.0 : 0.0;});
  const auto field = makeField(map);
  EXPECT_NEAR(field.getDistanceAt(Position3(-0.125, 0.0, 1.5)), 0.125, 1e-12);
  EXPECT_NEAR(field.getDistanceAt(Position3(-0.5, 0.0, 2.5)), std::sqrt(0.5),
              1e-12);
  EXPECT_LT(field.getDistanceAt(Position3(-0.125, 0.0, 1.5)) - 0.05 - 0.30,
            0.0);
  EXPECT_NEAR(field.getDistanceAt(Position3(0.1, 0.0, -0.4)),
              -std::hypot(0.1, 0.4), 1e-12);
  EXPECT_NEAR(field.getDistanceAt(Position3(-0.1, 0.0, -0.4)), -0.4, 1e-12);
  EXPECT_DOUBLE_EQ(field.getDistanceAt(Position3(0.0, 0.0, 1.0)), 0.0);
  for (double x : {-0.125, 0.0, 0.125, 0.25}) {
    const double left =
      field.getInterpolatedDistanceAt(Position3(x - 1e-7, 0.0, 2.25));
    const double right =
      field.getInterpolatedDistanceAt(Position3(x + 1e-7, 0.0, 2.25));
    EXPECT_LE(std::abs(right - left), 2.00001e-7);
  }
}

TEST(SignedDistanceField, TrenchHasNoBuriedInternalFaces) {
  auto map = makeMap();
  setHeights(map, [](const Position & point) {
      return std::abs(point.x()) < 0.5 ? 0.0 : 2.0;
  });
  const auto field = makeField(map);
  EXPECT_NEAR(field.getDistanceAt(Position3(0.0, 0.1, 1.0)), 0.5, 1e-12);
  EXPECT_NEAR(field.getDistanceAt(Position3(1.0, 0.1, 1.0)), -0.5, 1e-12);
  EXPECT_NEAR(field.getDistanceAt(Position3(1.0, 0.1, -0.5)), -std::sqrt(0.5),
              1e-12);
}

TEST(SignedDistanceField, UnknownCellsBlockClearanceAndRejectDirectQueries) {
  for (float unknown : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(),
      -std::numeric_limits<float>::infinity()})
  {
    auto map = makeMap(4.0, 0.25);
    setHeights(map, [unknown](const Position & point) {
        return point.x() >= 0.0 ? unknown : 0.0F;
    });
    const auto field = makeField(map);
    EXPECT_NEAR(field.getDistanceAt(Position3(-0.5, 0.0, 1.0)), 0.5, 1e-12);
    EXPECT_NEAR(field.getDistanceAt(Position3(-0.5, 0.0, 100.0)), 0.5, 1e-12);
    EXPECT_NEAR(field.getDistanceAt(Position3(-0.5, 0.0, -1.0)), -1.0, 1e-12);
    EXPECT_TRUE(field.isInside(Position3(-0.5, 0.0, 1.0)));
    EXPECT_FALSE(field.isInside(Position3(0.5, 0.0, 1.0)));
    EXPECT_THROW(field.getDistanceAt(Position3(0.5, 0.0, 1.0)),
                 std::out_of_range);
  }
}

TEST(SignedDistanceField,
     TranslatedAndRollingMapsMatchIndependentVolumeReference) {
  auto map = makeMap(4.0, 0.25);
  ASSERT_TRUE(map.move(Position(0.5, -0.75)));
  ASSERT_FALSE((map.getStartIndex() == 0).all());
  setHeights(map, [](const Position & point) {
      return 0.4 * std::floor(point.x() / 0.5) +
             0.2 * std::floor(point.y() / 0.5);
  });
  const auto field = makeField(map);
  auto normalized = map;
  normalized.convertToDefaultStartIndex();
  const auto other = makeField(normalized);
  std::mt19937 generator(317);
  std::uniform_real_distribution<double> xy(-1.7, 1.7);
  std::uniform_real_distribution<double> z(-2.0, 3.0);
  for (int sample = 0; sample < 500; ++sample) {
    const Position3 point(map.getPosition().x() + xy(generator),
      map.getPosition().y() + xy(generator), z(generator));
    const double expected = referenceDistance(map, point);
    EXPECT_NEAR(field.getDistanceAt(point), expected, 2e-12);
    EXPECT_NEAR(other.getDistanceAt(point), expected, 2e-12);
  }
}

TEST(SignedDistanceField,
     IrregularMapMatchesIndependentOccupiedAndFreeVolumeUnions) {
  auto map = makeMap(3.0, 0.25);
  std::mt19937 generator(919);
  std::uniform_real_distribution<float> heights(-0.5F, 1.5F);
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd();
    ++iterator)
  {
    map.at("elevation", *iterator) = heights(generator);
  }
  map.at("elevation", Index(2, 5)) = std::numeric_limits<float>::quiet_NaN();
  map.at("elevation", Index(8, 9)) = std::numeric_limits<float>::infinity();
  const auto field = makeField(map);
  std::uniform_real_distribution<double> xy(-1.45, 1.45);
  std::uniform_real_distribution<double> z(-1.0, 2.0);
  for (int sample = 0; sample < 1000; ++sample) {
    const Position3 point(xy(generator), xy(generator), z(generator));
    if (!field.isInside(point)) {
      continue;
    }
    const auto result = field.getDistanceAndGradientAt(point);
    EXPECT_NEAR(result.distance, referenceDistance(map, point), 2e-12);
    EXPECT_NEAR(result.gradient.norm(), 1.0, 2e-12);
  }
}

TEST(SignedDistanceField,
     CombinedGradientMatchesFiniteDifferencesAwayFromGeometricTies) {
  auto map = makeMap();
  setHeights(
      map, [](const Position & point) {return point.x() >= 0.0 ? 2.0 : 0.0;});
  const auto field = makeField(map);
  for (const Position3 & point :
    {Position3(-0.7, 0.17, 2.4), Position3(-0.7, 0.17, 1.4),
      Position3(0.7, 0.17, 1.4), Position3(0.1, 0.17, -0.4),
      Position3(3.7, 0.17, 3.0)})
  {
    SignedDistanceField::QueryStatistics statistics;
    const auto result = field.getDistanceAndGradientAt(point, &statistics);
    EXPECT_GT(statistics.nodesVisited, 0U);
    EXPECT_GT(statistics.facesTested, 0U);
    EXPECT_DOUBLE_EQ(result.distance, field.getDistanceAt(point));
    EXPECT_DOUBLE_EQ(result.distance, field.getInterpolatedDistanceAt(point));
    EXPECT_TRUE(result.gradient.isApprox(field.getDistanceGradientAt(point)));
    constexpr double step = 1e-6;
    for (int axis = 0; axis < 3; ++axis) {
      Position3 plus = point;
      Position3 minus = point;
      plus(axis) += step;
      minus(axis) -= step;
      const double finiteDifference =
        (field.getDistanceAt(plus) - field.getDistanceAt(minus)) /
        (2.0 * step);
      EXPECT_NEAR(result.gradient(axis), finiteDifference, 2e-8);
    }
  }
}

TEST(SignedDistanceField, InvalidInputFailsWithoutDestroyingPreviousBuild) {
  SignedDistanceField field;
  EXPECT_FALSE(field.isInside(Position3::Zero()));
  EXPECT_THROW(field.getDistanceAt(Position3::Zero()), std::logic_error);
  auto map = makeMap();
  map["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
  EXPECT_THROW(field.calculateSignedDistanceField(map, "elevation", 1.0),
               std::invalid_argument);
  map["elevation"].setZero();
  field.calculateSignedDistanceField(map, "elevation", 0.0);
  EXPECT_THROW(field.calculateSignedDistanceField(map, "missing", 1.0),
               std::invalid_argument);
  EXPECT_THROW(field.calculateSignedDistanceField(map, "elevation", -1.0),
               std::invalid_argument);
  EXPECT_THROW(field.calculateSignedDistanceField(
                   map, "elevation", std::numeric_limits<double>::infinity()),
               std::invalid_argument);
  EXPECT_DOUBLE_EQ(field.getDistanceAt(Position3(0.0, 0.0, 1.0)), 1.0);
  EXPECT_THROW(field.getDistanceAt(Position3(
                   std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0)),
               std::invalid_argument);
}

TEST(SignedDistanceField, SingleCellAndPointCloudUseActualCellCentres) {
  auto map = makeMap(0.25, 0.25);
  map.setPosition(Position(2.0, -3.0));
  map["elevation"].setConstant(0.5F);
  const auto field = makeField(map, 0.5);
  EXPECT_NEAR(field.getDistanceAt(Position3(2.0, -3.0, 0.55)), 0.05, 1e-12);
  EXPECT_NEAR(field.getDistanceAt(Position3(2.0, -3.0, -1.0)), -1.5, 1e-12);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  field.convertToPointCloud(cloud);
  ASSERT_EQ(cloud.size(), 3U);
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_FLOAT_EQ(cloud[i].x, 2.0F);
    EXPECT_FLOAT_EQ(cloud[i].y, -3.0F);
    EXPECT_FLOAT_EQ(cloud[i].z, static_cast<float>(0.5 + 0.25 * i));
    EXPECT_FLOAT_EQ(cloud[i].intensity, i == 0 ? 0.0F : 0.125F);
  }
}

}  // namespace

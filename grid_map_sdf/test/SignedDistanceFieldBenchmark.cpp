// Deterministic source benchmark; timing is reported, never used as a test
// threshold.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_sdf/SignedDistanceField.hpp"

int main()
{
  constexpr int cells = 240;
  constexpr double resolution = 0.25;
  constexpr int queries = 20000;
  for (const std::string kind : {"flat", "rough", "checkerboard"}) {
    grid_map::GridMap map({"elevation"});
    map.setGeometry(grid_map::Length(cells * resolution, cells * resolution),
                    resolution);
    std::mt19937 generator(431);
    std::uniform_real_distribution<float> noise(-0.2F, 0.2F);
    for (int row = 0; row < cells; ++row) {
      for (int column = 0; column < cells; ++column) {
        float height = 0.0F;
        if (kind == "rough") {
          height = 0.5 * std::sin(0.18 * row) + 0.3 * std::cos(0.2 * column) +
            noise(generator);
        } else if (kind == "checkerboard") {
          height = (row + column) % 2 == 0 ? 0.0F : 2.0F;
        }
        map.at("elevation", grid_map::Index(row, column)) = height;
      }
    }
    grid_map::SignedDistanceField field;
    const auto buildStart = std::chrono::steady_clock::now();
    field.calculateSignedDistanceField(map, "elevation", 5.0);
    const double buildMs = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - buildStart)
      .count();
    std::uniform_real_distribution<double> xy(-29.0, 29.0);
    std::uniform_real_distribution<double> z(-1.0, 5.0);
    std::vector<double> samples;
    samples.reserve(queries);
    std::size_t sumFaces = 0;
    std::size_t maxFaces = 0;
    std::size_t maxNodes = 0;
    double checksum = 0.0;
    for (int query = 0; query < queries; ++query) {
      const grid_map::Position3 point(xy(generator), xy(generator),
        z(generator));
      grid_map::SignedDistanceField::QueryStatistics statistics;
      const auto start = std::chrono::steady_clock::now();
      const auto result = field.getDistanceAndGradientAt(point, &statistics);
      samples.push_back(std::chrono::duration<double, std::micro>(
                            std::chrono::steady_clock::now() - start)
        .count());
      checksum += result.distance + result.gradient.x();
      sumFaces += statistics.facesTested;
      maxFaces = std::max(maxFaces, statistics.facesTested);
      maxNodes = std::max(maxNodes, statistics.nodesVisited);
    }
    std::sort(samples.begin(), samples.end());
    std::cout << kind << " cells=" << cells << "x" << cells
              << " resolution=" << resolution << " build_ms=" << buildMs
              << " queries=" << queries
              << " query_us_p50=" << samples[queries / 2]
              << " query_us_p95=" << samples[queries * 95 / 100]
              << " query_us_p99=" << samples[queries * 99 / 100]
              << " query_us_max=" << samples.back()
              << " faces_mean=" << static_cast<double>(sumFaces) / queries
              << " faces_max=" << maxFaces << " nodes_max=" << maxNodes
              << " checksum=" << checksum << '\n';
  }
}

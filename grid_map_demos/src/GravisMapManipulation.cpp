/*
 * GravisMapManipulation.cpp
 *
 */

#include "grid_map_demos/GravisMapManipulation.hpp"

#include <opencv2/imgcodecs.hpp>
#include <grid_map_core/grid_map_core.hpp>

namespace grid_map_demos {

GravisMapManipulation::GravisMapManipulation(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  readParameters();
  gridMapSubscriber_ = nodeHandle_.subscribe("/planning_map", 1, &GravisMapManipulation::gridMapCallback, this);
  post_digging_map_pub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(
      "/gravis_map", 1);
}

GravisMapManipulation::~GravisMapManipulation()=default;

void GravisMapManipulation::readParameters()
{
  nodeHandle_.param("new_center_x", pos_.x(), -27.8);
  nodeHandle_.param("new_center_y", pos_.y(), -48.68);
  nodeHandle_.param("new_length_x", len_.x(), 17.0);
  nodeHandle_.param("new_length_y", len_.y(), 18.9);
  nodeHandle_.param("interpolation_alpha", interpolation_alpha_, 0.39);
}

void GravisMapManipulation::gridMapCallback(const grid_map_msgs::GridMap& msg)
{
  ROS_INFO("Received!.");
  readParameters();
  bool isSuccess;
  grid_map::GridMap inputGridMap;

  // Crop
  grid_map::GridMapRosConverter::fromMessage(msg, inputGridMap);
  grid_map::GridMap subMap = inputGridMap.getSubmap(pos_, len_, isSuccess);

  // Smooth out final map by including planned map
  subMap.add("post_digging");
  subMap.add("pre_digging");
  subMap["pre_digging"] = subMap["original_elevation"];
  for (grid_map::GridMapIterator iterator(subMap); !iterator.isPastEnd(); ++iterator) {
    subMap.at("post_digging", *iterator) = interpolation_alpha_ * subMap.at("elevation", *iterator) +
      (1.0 - interpolation_alpha_) * subMap.at("desired_elevation", *iterator);
  }

  // Keep hill on the top lef corner from real scanning to have full elevation
  for (grid_map::CircleIterator iterator(subMap, grid_map::Position(-18.08, -48.6), 2.9);
       !iterator.isPastEnd(); ++iterator) {
    subMap.at("post_digging", *iterator) = subMap.at("elevation", *iterator);
  }
  // Remove Lorenzo's car
  double alpha = 0.7;
  for (grid_map::PolygonIterator iterator(subMap, grid_map::Polygon(std::vector<grid_map::Position>{
          {-37.6, -48.2}, {-40.66, -47.16}, {-41.67, -51.14}, {-39.03, -52.34}}));
       !iterator.isPastEnd(); ++iterator) {
         subMap.at("post_digging", *iterator) = alpha * subMap.at("elevation", *iterator) +
           (1.0 - alpha) * subMap.at("desired_elevation", *iterator);
         subMap.at("pre_digging", *iterator) = subMap.at("post_digging", *iterator);
  }

  /*
  // Filter points close on top right edge of the digging area
  alpha = 0.15;
  for (grid_map::CircleIterator iterator(subMap, grid_map::Position(-21.53, -57.69), 4.2);
       !iterator.isPastEnd(); ++iterator) {
         subMap.at("post_digging", *iterator) = alpha * subMap.at("elevation", *iterator) +
           (1.0 - alpha) * subMap.at("desired_elevation", *iterator);
  }*/

  grid_map_msgs::GridMap outMsg;
  grid_map::GridMapRosConverter::toMessage(subMap, outMsg);
  post_digging_map_pub_.publish(outMsg);
}

} /* namespace */

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
  grid_map_pub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(
      "/gravis_map", 1);
}

GravisMapManipulation::~GravisMapManipulation()=default;

void GravisMapManipulation::readParameters()
{
}

void GravisMapManipulation::gridMapCallback(const grid_map_msgs::GridMap& msg)
{
  ROS_INFO("Received!.");

  grid_map::Position pos(-28.17, -48.38);
  grid_map::Length len(17.0, 18.0);
  double alpha = 0.39;
  bool isSuccess;
  grid_map::GridMap inputGridMap;


  // Crop
  grid_map::GridMapRosConverter::fromMessage(msg, inputGridMap);
  grid_map::GridMap subMap = inputGridMap.getSubmap(pos, len, isSuccess);

  // Filter
  subMap.add("gravis");
  for (grid_map::GridMapIterator iterator(subMap); !iterator.isPastEnd(); ++iterator) {
    subMap.at("gravis", *iterator) = alpha * subMap.at("elevation", *iterator) + (1.0 - alpha) * subMap.at("desired_elevation", *iterator);
  }

  grid_map_msgs::GridMap outMsg;
  grid_map::GridMapRosConverter::toMessage(subMap, outMsg);
  grid_map_pub_.publish(outMsg);

  ros::shutdown();
}

} /* namespace */

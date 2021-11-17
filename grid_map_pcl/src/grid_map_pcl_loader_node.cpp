/*
 * grid_map_pcl_loader_node.cpp
 *
 *  Created on: Aug 26, 2019
 *      Author: Edo Jelavic
 *      Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include "grid_map_pcl/GridMapPclLoader.hpp"
#include "grid_map_pcl/helpers.hpp"

namespace gm = ::grid_map::grid_map_pcl;

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map_pcl_loader_node");
  ros::NodeHandle nh("~");
  gm::setVerbosityLevelToDebugIfFlagSet(nh);

  grid_map::GridMapPclLoader gridMapPclLoader(nh);

  gridMapPclLoader.loadParameters(gridMapPclLoader.getParameterPath());

  // run
  ros::spin();
  return EXIT_SUCCESS;
}

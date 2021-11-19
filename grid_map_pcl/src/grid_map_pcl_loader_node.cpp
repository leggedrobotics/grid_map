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

  // Publisher for rviz
  ros::Publisher gridMapPub;
  gridMapPub = nh.advertise<grid_map_msgs::GridMap>("/loam/map_surface_grid_from_pcd", 1, true);

  // Object
  grid_map::GridMapPclLoader gridMapPclLoader(nh);

  // Load params
  gridMapPclLoader.loadParameters(gridMapPclLoader.getParameterPath());

  // Load point_cloud from pcd file
  const std::string pathToCloud = gm::getPcdFilePath(nh);
  gridMapPclLoader.loadCloudFromPcdFile(pathToCloud);

  // Convert to grid map
  gm::processPointcloud(&gridMapPclLoader, nh);
  grid_map::GridMap gridMap = gridMapPclLoader.getGridMap();
  gridMap.setFrameId(gm::getMapFrame(nh));

  // Post processing (inpainting and interpolation)
  grid_map::GridMap ouputGridMap = gridMapPclLoader.postProcessGridMap(gridMap);

  // Save to bag file
  gm::saveGridMap(ouputGridMap, nh, gm::getMapRosbagTopic(nh));

  // publish grid map
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(ouputGridMap, msg);
  gridMapPub.publish(msg);

  // run
  ros::spin();
  return EXIT_SUCCESS;
}

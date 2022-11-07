/*
 * grid_map_to_image_demo.cpp
 *
 *  Created on: October 19, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "grid_map_demos/GravisMapManipulation.hpp"

#include <ros/ros.h>

/*
 * Usage:
 * $ rosrun grid_map_demos grid_map_to_image_demo _grid_map_topic:=/grid_map _file:=/home/$USER/Desktop/grid_map_image.png
 */
int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "grid_map_to_image_demo");
  ros::NodeHandle nh("~");
  grid_map_demos::GravisMapManipulation gravisMapManipulation(nh);

  ros::spin();
  return EXIT_SUCCESS;
}

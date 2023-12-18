#include <iostream>
#include "ros/ros.h"

#include <vector>
#include <string>

#include "route_verbalization/mapManipulators/MapReader.h"
#include "route_verbalization/mapManipulators/MapDrawer.h"

ros::NodeHandle* n_;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semantic_route_drawer");

  ros::NodeHandle n;
  n_ = &n;

  ros::service::waitForService("/ontologenius/reasoner", -1);

  ROS_DEBUG("semantic_route_drawer ready");

  MapReader reader;
  reader.getMap();
  MapDrawer drawer;
  drawer.draw(reader.corridors());
  drawer.draw(reader.openspaces());

  ROS_DEBUG("KILL semantic_route_drawer");

  return 0;
}

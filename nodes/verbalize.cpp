
#include <ros/ros.h>

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include "route_verbalization/VerbalizeRegionRoute.h"

#include "route_verbalization/Verbalizers/Verbalizer.h"

ros::NodeHandle* n_;
Verbalizer* verbalizer;

bool regionHandle(route_verbalization::VerbalizeRegionRoute::Request  &req,
                  route_verbalization::VerbalizeRegionRoute::Response  &res)
{
  std::string text;

  res.success = verbalizer->regionRoute(req.route, req.start_place, text);
  res.region_route = text;

  return true;
}

bool placeHandle(route_verbalization::VerbalizeRegionRoute::Request  &req,
                 route_verbalization::VerbalizeRegionRoute::Response  &res)
{
  std::string text;

  res.success = verbalizer->placeRoute(req.route, req.start_place, req.goal_shop, text);
  res.region_route = text;

  return true;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "verbalize");

  ros::NodeHandle n;
  n_ = &n;

  verbalizer = new Verbalizer;
  verbalizer->init();

  ros::ServiceServer region_route_service = n.advertiseService("route_verbalization/verbalizeRegion", regionHandle);
  ros::ServiceServer place_route_service = n.advertiseService("route_verbalization/verbalizePlace", placeHandle);

  ros::spin();

  delete verbalizer;

  return 0;
}

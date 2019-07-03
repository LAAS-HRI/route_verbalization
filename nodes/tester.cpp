#include <iostream>
#include "ros/ros.h"


#include "semantic_route_description/SemanticRoute.h"
#include "route_verbalization/VerbalizeRegionRoute.h"

//#include "pepper_msg/Say.h"

//roslaunch pepper_bringup pepper_full.launch nao_ip:=mummer.laas.fr roscore_ip:=127.0.0.1 network_interface:=enp0s31f6

ros::NodeHandle* n_;

void say(std::string text)
{
  std::cout << "[SAY] " << text << std::endl;
  /*ros::ServiceClient client = n_->serviceClient<pepper_msg::Say>("/naoqi_driver/animated_speech/say");

  pepper_msg::Say srv;
  srv.request.text = text;

  if(!client.call(srv))
    std::cout << "[FAILED SAY]" << std::endl;*/
}

void getRoutesRegion(std::vector<std::vector<std::string> >& routes, std::vector<float>& costs, std::string from, std::string to, std::string persona, bool sign)
{
  ros::ServiceClient client = n_->serviceClient<semantic_route_description::SemanticRoute>("semantic_route_description/get_route_region");

  semantic_route_description::SemanticRoute srv;
  srv.request.from = from;
  srv.request.to = to;
  srv.request.persona = persona;
  srv.request.signpost = sign;

  if(client.call(srv))
  {
    costs = srv.response.costs;
    for(size_t i = 0; i < srv.response.routes.size(); i++)
      routes.push_back(srv.response.routes[i].route);
  }
}

void getRoutesPlace(std::vector<std::vector<std::string> >& routes, std::vector<float>& costs, std::string from, std::string to, std::string persona, bool sign)
{
  ros::ServiceClient client = n_->serviceClient<semantic_route_description::SemanticRoute>("semantic_route_description/get_route");

  semantic_route_description::SemanticRoute srv;
  srv.request.from = from;
  srv.request.to = to;
  srv.request.persona = persona;
  srv.request.signpost = sign;

  if(client.call(srv))
  {
    costs = srv.response.costs;
    for(size_t i = 0; i < srv.response.routes.size(); i++)
      routes.push_back(srv.response.routes[i].route);
  }
}

std::vector<std::string> getRoute(std::string from, std::string to, std::string persona, bool sign, bool region = true)
{
  std::vector<std::vector<std::string> > routes;
  std::vector<float> costs;
  if(region)
    getRoutesRegion(routes, costs, from, to, persona, sign);
  else
    getRoutesPlace(routes, costs, from, to, persona, sign);


  double min_cost = 1000;
  int index = -1;
  for(size_t i = 0; i < costs.size(); i++)
  {
    if(min_cost > costs[i])
    {
      min_cost = costs[i];
      index = i;
    }
  }

  if(index != -1)
    return routes[index];
  else
  {
    std::vector<std::string> none;
    return none;
  }
}

bool regionHandle(semantic_route_description::SemanticRoute::Request  &req,
                  semantic_route_description::SemanticRoute::Response  &res)
{
  std::vector<std::string> route = getRoute(req.from, req.to, req.persona, req.signpost);

  ros::ServiceClient client = n_->serviceClient<route_verbalization::VerbalizeRegionRoute>("route_verbalization/verbalizeRegion");

  route_verbalization::VerbalizeRegionRoute srv;
  srv.request.route = route;
  srv.request.start_place = req.from;
  srv.request.goal_shop = req.to;

  if(client.call(srv))
  {
    std::cout << srv.response.region_route << std::endl;
    res.goals.push_back(srv.response.region_route);
  }

  return true;
}

bool placeHandle(semantic_route_description::SemanticRoute::Request  &req,
                  semantic_route_description::SemanticRoute::Response  &res)
{
  std::vector<std::string> route = getRoute(req.from, req.to, req.persona, req.signpost, false);

  ros::ServiceClient client = n_->serviceClient<route_verbalization::VerbalizeRegionRoute>("route_verbalization/verbalizePlace");

  route_verbalization::VerbalizeRegionRoute srv;
  srv.request.route = route;
  srv.request.start_place = req.from;
  srv.request.goal_shop = req.to;

  if(client.call(srv))
  {
    std::cout << srv.response.region_route << std::endl;
    res.goals.push_back(srv.response.region_route);
    say(srv.response.region_route);
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "semantic_route_drawer");

  ros::NodeHandle n;
  n_ = &n;

  ros::ServiceServer region_service = n.advertiseService("route_verbalization/testRegion", regionHandle);
  ros::ServiceServer place_service = n.advertiseService("route_verbalization/testPlace", placeHandle);

  ros::spin();

  return 0;
}

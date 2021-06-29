#ifndef VERBALIZER_H
#define VERBALIZER_H

#include <ros/ros.h>
#include "ontologenius/OntologyManipulator.h"

#include "route_verbalization/Verbalizers/RegionVerbalizer.h"
#include "route_verbalization/Verbalizers/PlaceVerbalizer.h"

class Verbalizer
{
public:
  Verbalizer(ros::NodeHandle* n) : n_(n), onto_(n_), region_verbalizer(&onto_), place_verbalizer(&onto_) {}
  ~Verbalizer() {}

  void init() {place_verbalizer.init(n_);}

  bool regionRoute(std::vector<std::string> route, std::string start_place, std::string& text)
  {
    return region_verbalizer.verbalizeRegionRoute(route, start_place, text);
  }

  bool placeRoute(std::vector<std::string> route, std::string start_place, std::string goal_shop, std::string& text)
  {
    return place_verbalizer.verbalizePlaceRoute(route, start_place, goal_shop, text);
  }

private:
  ros::NodeHandle* n_;
  OntologyManipulator onto_;
  RegionVerbalizer region_verbalizer;
  PlaceVerbalizer place_verbalizer;
};

#endif

#ifndef PLACEVERBALIZER_H
#define PLACEVERBALIZER_H

#include "ontoloGenius/utility/OntologyManipulator.h"
#include "route_verbalization/mapManipulators/MapReader.h"

#include <ros/ros.h>
#include <string>
#include <map>
#include <vector>

class PlaceVerbalizer
{
public:
  PlaceVerbalizer(OntologyManipulator* onto);
  ~PlaceVerbalizer() {}

  void init(ros::NodeHandle* n);

  bool verbalizePlaceRoute(std::vector<std::string> route, std::string start_place, std::string goal_shop, std::string& text);

private:
  OntologyManipulator* onto_;

  std::vector<corridor_t> corridors_;
  std::vector<openspace_t> openspaces_;
  std::map<std::string, std::string> word_map;

  std::string getDirection(std::string& from, std::string& path, std::string& to);
  std::string getDirectionPath(std::string& from, std::string& path, std::string& to);
  std::string getDirectionCorridor(std::string& from, std::string& corridor_name, std::string& to);
  std::string getDirectionCorridor(std::string& from, corridor_t& corridor, std::string& to);
  std::string getDirectionOpenspace(std::string& from, std::string& openspace_name, std::string& to);
  std::string getDirectionOpenspace(std::string& from, openspace_t& openspace, std::string& to);
  std::string getDirectionRegion(std::string& from, std::string& region_name, std::string& to);

  int getIndex(std::vector<std::string>& vect, std::string& word);
  bool isBefore(std::vector<std::string>& vect, std::string& word, size_t index);
};

#endif

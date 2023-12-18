#ifndef MAPREADER_H
#define MAPREADER_H

#include <string>
#include <vector>
#include <map>

#include "ros/ros.h"
#include "ontologenius/OntologyManipulator.h"

struct corridor_t
{
  std::string name_;
  std::vector<std::string> at_right_;
  std::vector<std::string> at_left_;
  std::vector<std::string> at_begin_edge_;
  std::vector<std::string> at_end_edge_;
  std::map<std::string, std::string> in_front_of_;
};

struct openspace_t
{
  std::string name_;
  std::vector<std::string> around_;
  std::map<std::string, std::string> in_front_of_;
};

class MapReader
{
public:
  MapReader() { onto_.close(); }
  ~MapReader() {}

  void getMap();

  std::vector<corridor_t> corridors() {return corridors_; }
  std::vector<openspace_t> openspaces() {return openspaces_; }

private:
  onto::OntologyManipulator onto_;

  std::vector<corridor_t> corridors_;
  std::vector<openspace_t> openspaces_;

  void getCorridors();
  void getInFrontOf(corridor_t& corridor);
  void getSides(corridor_t& corridor);
  std::vector<std::string> getSide(std::string corridor, std::string property);
  void displayCorridor(corridor_t corridor);

  void getOpenspaces();
  void getInFrontOf(openspace_t& openspace);
  void getAround(openspace_t& openspace);
  void displayOpenspaces(openspace_t openspace);

  std::vector<std::string> orderPlaces(std::vector<std::string> base, bool inverse);
  void displaySide(std::vector<std::string> side, std::string name);
};

#endif

#ifndef REGIONVERBALIZER_H
#define REGIONVERBALIZER_H

#include "ontologenius/OntologyManipulator.h"

#include <string>
#include <map>
#include <vector>

class RegionVerbalizer
{
public:
  RegionVerbalizer(onto::OntologyManipulator* onto);
  ~RegionVerbalizer() {}

  bool verbalizeRegionRoute(std::vector<std::string> route, std::string start_place, std::string& text);

private:
  onto::OntologyManipulator* onto_;

  std::map<std::string, std::string> word_map;

  void initWordMaps();
};

#endif

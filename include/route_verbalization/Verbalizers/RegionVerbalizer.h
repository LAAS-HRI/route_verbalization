#ifndef REGIONVERBALIZER_H
#define REGIONVERBALIZER_H

#include "ontoloGenius/utility/OntologyManipulator.h"

#include <string>
#include <map>
#include <vector>

class RegionVerbalizer
{
public:
  RegionVerbalizer(OntologyManipulator* onto);
  ~RegionVerbalizer() {}

  bool verbalizeRegionRoute(std::vector<std::string> route, std::string start_place, std::string& text);

private:
  OntologyManipulator* onto_;

  std::map<std::string, std::string> word_map;

  void initWordMaps();
};

#endif

#include "route_verbalization/Verbalizers/RegionVerbalizer.h"

RegionVerbalizer::RegionVerbalizer(OntologyManipulator* onto) : onto_(onto)
{
  initWordMaps();
}

void RegionVerbalizer::initWordMaps()
{
  word_map["elevator"] = "take the ";
  word_map["stair"] = "take the ";
  word_map["escalator"] = "take the ";
}

bool RegionVerbalizer::verbalizeRegionRoute(std::vector<std::string> route, std::string start_place, std::string& text)
{
  if(route.size() != 0)
  {
    if(route.size() > 1)
    {
      std::vector<std::string> from_type = onto_->individuals.getUp(start_place);
      if(std::find(from_type.begin(), from_type.end(), "infodesk") == from_type.end())
        text += "from " + onto_->individuals.getName(start_place) + " ";

      for(size_t i = 1; i < route.size(); i = i + 2)
      {
        if(i + 2 >= route.size() && (i != 1))
          text += "and then ";
        else if(i != 1)
          text += "then ";

        std::vector<std::string> interfaces = onto_->individuals.getUp(route[i], 1);
        std::string interface_name = "";
        std::string verb = "go through the ";
        for(size_t interface = 0; interface < interfaces.size(); interface++)
            if (word_map.find(interfaces[interface]) != word_map.end())
            {
              verb = word_map[interfaces[interface]];
              interface_name = interfaces[interface];
            }

        if(interface_name == "")
          interface_name = onto_->individuals.getName(route[i]);

        text += verb + interface_name + " ";

        if(i + 2 >= route.size())
        {
          text += "to get to ";
          text += "the " + onto_->individuals.getName(route[i + 1]) + " ";
        }
      }
    }

    return true;
  }
  else
    return false;
}

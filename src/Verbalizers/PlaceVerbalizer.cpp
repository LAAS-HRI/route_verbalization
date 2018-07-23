#include "route_verbalization/Verbalizers/PlaceVerbalizer.h"

#include <iostream>

PlaceVerbalizer::PlaceVerbalizer(OntologyManipulator* onto) : onto_(onto)
{
  word_map["elevator"] = "take the ";
  word_map["stair"] = "take the ";
  word_map["escalator"] = "take the ";
  word_map["corridor"] = "take the ";
  word_map["walkway"] = "take the ";
  word_map["pathIntersection"] = "take the ";
}

void PlaceVerbalizer::init(ros::NodeHandle* n)
{
  MapReader reader(n);
  reader.getMap();

  corridors_ = reader.corridors();
  openspaces_ = reader.openspaces();
}

bool PlaceVerbalizer::verbalizePlaceRoute(std::vector<std::string> route, std::string start_place, std::string goal_shop, std::string& text)
{
  std::cout << "*******************" << std::endl;
  if(route.size() != 0)
  {
    if(route.size() > 1)
    {
      size_t start_index = 1;
      std::vector<std::string> from_type = onto_->individuals.getUp(start_place);
      if(std::find(from_type.begin(), from_type.end(), "infodesk") == from_type.end())
      {
        text += "from " + onto_->individuals.getName(start_place) + " ";
        //ground
        start_index = 1;
      }
      else
        start_index = 3;

      for(size_t i = start_index; i < route.size(); i = i + 2)
      {
        if(i + 2 > route.size() && (i != 1))
          text += "and then ";
        else if(i + 2 < route.size() && (i != 1))
          text += "then ";

        if(i + 2 >= route.size()) // goal shop
        {
          text += "to get to ";
          text += onto_->individuals.getName(route[i + 1]) + " ";
        }
        else
        {
          /*std::vector<std::string> interfaces = onto_->getUp(route[i + 1], 1);
          std::string interface_name = "";
          std::string verb = "go through the ";
          for(size_t interface = 0; interface < interfaces.size(); interface++)
              if (word_map.find(interfaces[interface]) != word_map.end())
              {
                verb = word_map[interfaces[interface]];
                if(onto_->getUp(route[i + 1], -1, "corridor").size())
                  interface_name = "corridor";
                else if(onto_->getUp(route[i + 1], -1, "pathIntersection").size())
                  interface_name = "corridor";
                else
                  interface_name = interfaces[interface];
              }

          if(interface_name == "")
            interface_name = onto_->getName(route[i + 1]);

          text += verb + interface_name + " ";*/
        }

        std::cout << "from " << route[i - 1] << " to " << route[i + 1] << " by " << route[i] << std::endl;
        std::string direction = getDirection(route[i - 1], route[i], route[i + 1]);
        std::cout << "dir = " << direction << std::endl;
      }
    }

    return true;
  }
  else
    return false;
}

std::string PlaceVerbalizer::getDirection(std::string& from, std::string& path, std::string& to)
{
  if(onto_->individuals.getUp(path, -1, "path").size())
    return getDirectionPath(from, path, to);
  else if(onto_->individuals.getUp(path, -1, "region").size())
    return getDirectionRegion(from, path, to);
  else
    return "";
}

std::string PlaceVerbalizer::getDirectionPath(std::string& from, std::string& path, std::string& to)
{
  if(onto_->individuals.getUp(path, -1, "corridor").size())
    return getDirectionCorridor(from, path, to);
  else if(onto_->individuals.getUp(path, -1, "openspace").size())
    return getDirectionOpenspace(from, path, to);
  else
    return "";
}

std::string PlaceVerbalizer::getDirectionCorridor(std::string& from, std::string& corridor_name, std::string& to)
{
  std::string res;
  corridor_t corridor;

  for(corridor_t& cor : corridors_)
    if(cor.name_ == corridor_name)
    {
      corridor = cor;
      break;
    }

  if(corridor.name_ != "")
    res = getDirectionCorridor(from, corridor, to);

  return res;
}

std::string PlaceVerbalizer::getDirectionCorridor(std::string& from, corridor_t& corridor, std::string& to)
{
  std::string res = "none";

  int from_index, to_index = -1;
  if((to_index = getIndex(corridor.at_begin_edge_, to)) >= 0) //next goal at begin_edge
  {
    if((from_index = getIndex(corridor.at_begin_edge_, from)) >= 0)
    {
      if(isBefore(corridor.at_begin_edge_, from, to_index))
        res = "at your right";
      else
        res = "at your left";
    }
    else if((from_index = getIndex(corridor.at_right_, from)) >= 0)
      res = "turn left and go to the end of the corridor";
    else if((from_index = getIndex(corridor.at_left_, from)) >= 0)
      res = "turn right and go to the end of the corridor";
    else
      res = "at the end of the corridor";
  }
  else if((to_index = getIndex(corridor.at_end_edge_, to)) >= 0) //next goal at end_edge
  {
    if((from_index = getIndex(corridor.at_end_edge_, from)) >= 0)
    {
      if(isBefore(corridor.at_begin_edge_, from, to_index))
        res = "at your right";
      else
        res = "at your left";
    }
    else if((from_index = getIndex(corridor.at_right_, from)) >= 0)
      res = "turn right and go to the end of the corridor";
    else if((from_index = getIndex(corridor.at_left_, from)) >= 0)
      res = "turn left and go to the end of the corridor";
    else
      res = "at the end of the corridor";
  }
  else if((to_index = getIndex(corridor.at_right_, to)) >= 0) //next goal at right
  {
    if((from_index = getIndex(corridor.at_right_, from)) >= 0)
    {
      if(isBefore(corridor.at_right_, from, to_index))
        res = "turn at right and it will be at your right";
      else
        res = "turn at left and it will be at your left";
    }
    else if((from_index = getIndex(corridor.at_end_edge_, from)) >= 0)
      res = "at your left";
    else if((from_index = getIndex(corridor.at_begin_edge_, from)) >= 0)
      res = "at your right";
    else // from at left
    {
      if(corridor.in_front_of_.find(from) != corridor.in_front_of_.end())
      {
        if(corridor.in_front_of_[from] == to)
          res = "in front of you";
        else if(getIndex(corridor.at_left_, corridor.in_front_of_[from]) > getIndex(corridor.at_left_, to))
          res = "turn right and it will be at your left";
        else
          res = "turn left and it will be at your right";
      }
      else
        std::cout << "Your programmer is bugging ..." << std::endl; //TODO
    }
  }
  else if((to_index = getIndex(corridor.at_left_, to)) >= 0) //next goal at left
  {
    if((from_index = getIndex(corridor.at_left_, from)) >= 0)
    {
      if(isBefore(corridor.at_left_, from, to_index))
        res = "turn at right and it will be at your right";
      else
        res = "turn at left and it will be at your left";
    }
    else if((from_index = getIndex(corridor.at_end_edge_, from)) >= 0)
      res = "at your right";
    else if((from_index = getIndex(corridor.at_begin_edge_, from)) >= 0)
      res = "at your left";
    else // from at left
    {
      if(corridor.in_front_of_.find(from) != corridor.in_front_of_.end())
      {
        if(corridor.in_front_of_[from] == to)
          res = "in front of you";
        else if(getIndex(corridor.at_left_, corridor.in_front_of_[from]) > getIndex(corridor.at_left_, to))
          res = "turn right and it will be at your left";
        else
          res = "turn left and it will be at your right";
      }
      else
        std::cout << "Your programmer is bugging ..." << std::endl; //TODO
    }
  }

  return res;
}

std::string PlaceVerbalizer::getDirectionOpenspace(std::string& from, std::string& openspace_name, std::string& to)
{
  std::string res;
  openspace_t openspace;

  for(openspace_t& os : openspaces_)
    if(os.name_ == openspace_name)
    {
      openspace = os;
      break;
    }

  if(openspace.name_ != "")
    res = getDirectionOpenspace(from, openspace, to);

  return res;
}

std::string PlaceVerbalizer::getDirectionOpenspace(std::string& from, openspace_t& openspace, std::string& to)
{
  std::string res = "os none";
  int from_index = getIndex(openspace.around_, from);
  if(from_index >= 0)
  {
    if(openspace.in_front_of_.find(openspace.around_[from_index]) != openspace.in_front_of_.end())
    {
      if(openspace.in_front_of_[openspace.around_[from_index]] == to)
        res = "just in front";
      else
      {
        for(size_t i = 0; i < openspace.around_.size(); i++)
        {
          if(openspace.around_[(from_index + i) % openspace.around_.size()] == to)
          {
            res = "at your left";
            break;
          }
          else if(openspace.around_[(from_index + i) % openspace.around_.size()] == openspace.in_front_of_[openspace.around_[from_index]])
          {
            res = "at your right";
            break;
          }
        }
      }
    }
    else if(openspace.around_[(from_index + 1) % openspace.around_.size()] == to)
      res = "at left";
    else
    {
      size_t index = from_index - 1;
      if(from_index == 0)
        index = openspace.around_.size() - 1;
      if(openspace.around_[index] == to)
        res = "at right";
      else
      {
        int to_index = getIndex(openspace.around_, to);
        if(to_index >= 0)
          res = "that is next to the " + onto_->individuals.getName(openspace.around_[(to_index + 1) % openspace.around_.size()]);
      }
    }
  }
  return res;
}

std::string PlaceVerbalizer::getDirectionRegion(std::string& from, std::string& region_name, std::string& to)
{
  std::string res;

  return res;
}

int PlaceVerbalizer::getIndex(std::vector<std::string>& vect, std::string& word)
{
  int index = -1;
  for(size_t i = 0; i < vect.size(); i++)
    if(vect[i] == word)
    {
      index = i;
      break;
    }
  return index;
}

bool PlaceVerbalizer::isBefore(std::vector<std::string>& vect, std::string& word, size_t index)
{
  for(size_t i = 0; i < index; i++)
    if(vect[i] == word)
      return true;
  return false;
}

#include "route_verbalization/Verbalizers/PlaceVerbalizer.h"

#include <iostream>
#include <random>

PlaceVerbalizer::PlaceVerbalizer(OntologyManipulator* onto) : onto_(onto), sentences(onto_)
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
      bool from_current = true;
      size_t start_index = 1;
      std::vector<std::string> from_type = onto_->individuals.getUp(start_place);
      if(std::find(from_type.begin(), from_type.end(), "infodesk") == from_type.end())
      {
        text += "from " + onto_->individuals.getName(start_place) + " ";
        //ground
        start_index = 1;
        from_current = false;
      }
      else
        start_index = 1;

      for(size_t i = start_index; i < route.size(); i = i + 2)
      {
        size_t nb_steps = (route.size() - start_index) / 2;
        size_t step = (i - start_index)/2 + 1;

        std::cout << "from " << route[i - 1] << " to " << route[i + 1] << " by " << route[i] << std::endl;
        std::vector<sentence_req_t> req = getDirection(route[i - 1], route[i], route[i + 1], from_current, step, nb_steps);
        for(size_t seq = 0; seq < req.size(); seq++)
        {
          if((req.size() > 1) && (seq == 0) && (step > 1))
            text += "and, ";
          else if((step > 1) && (step < nb_steps))
          {
            text += sentences.getPunctuation();
            std::cout << "ponctuation" << std::endl;
          }

          std::string tmp;
          if((step == 1) && (nb_steps == 1))
            tmp = sentences.getSentence(req[seq], true);
          else
            tmp = sentences.getSentence(req[seq]);

          text += tmp;
        }
      }
    }

    return true;
  }
  else
    return false;
}

std::vector<sentence_req_t> PlaceVerbalizer::getDirection(std::string& from, std::string& path, std::string& to, bool from_current, size_t step, size_t nb_steps)
{
  if(onto_->individuals.getUp(path, -1, "path").size())
    return getDirectionPath(from, path, to, from_current, step, nb_steps);
  else if(onto_->individuals.getUp(path, -1, "region").size())
    return getDirectionRegion(from, path, to, from_current, step, nb_steps);
  else
  {
    std::vector<sentence_req_t> tmp;
    return tmp;
  }
}

std::vector<sentence_req_t> PlaceVerbalizer::getDirectionPath(std::string& from, std::string& path, std::string& to, bool from_current, size_t step, size_t nb_steps)
{
  if(onto_->individuals.getUp(path, -1, "corridor").size())
    return getDirectionCorridor(from, path, to, from_current, step, nb_steps);
  else if(onto_->individuals.getUp(path, -1, "openspace").size())
    return getDirectionOpenspace(from, path, to, from_current, step, nb_steps);
  else
  {
    std::vector<sentence_req_t> tmp;
    return tmp;
  }
}

std::vector<sentence_req_t> PlaceVerbalizer::getDirectionCorridor(std::string& from, std::string& corridor_name, std::string& to, bool from_current, size_t step, size_t nb_steps)
{
  std::vector<sentence_req_t> res;
  corridor_t corridor;

  for(corridor_t& cor : corridors_)
    if(cor.name_ == corridor_name)
    {
      corridor = cor;
      break;
    }

  if(corridor.name_ != "")
    res = getDirectionCorridor(from, corridor, to, from_current, step, nb_steps);

  return res;
}

std::vector<sentence_req_t> PlaceVerbalizer::getDirectionCorridor(std::string& from, corridor_t& corridor, std::string& to, bool from_current, size_t step, size_t nb_steps)
{
  sentence_req_t res_prelim = sentence_req_t(none_type, "");
  sentence_req_t res;

  right_from = "";
  left_from = "";
  right_to = "";
  left_to = "";

  if(getIndex(corridor.at_begin_edge_, from) >= 0)
    getRightLeft(corridor.at_begin_edge_, from, right_from, left_from);
  else if(getIndex(corridor.at_end_edge_, from) >= 0)
    getRightLeft(corridor.at_end_edge_, from, right_from, left_from);
  else if(getIndex(corridor.at_right_, from) >= 0)
    getRightLeft(corridor.at_right_, from, right_from, left_from);
  else if(getIndex(corridor.at_left_, from) >= 0)
    getRightLeft(corridor.at_left_, from, right_from, left_from);

  if(getIndex(corridor.at_begin_edge_, to) >= 0)
    getRightLeft(corridor.at_begin_edge_, to, right_to, left_to);
  else if(getIndex(corridor.at_end_edge_, to) >= 0)
    getRightLeft(corridor.at_end_edge_, to, right_to, left_to);
  else if(getIndex(corridor.at_right_, to) >= 0)
    getRightLeft(corridor.at_right_, to, right_to, left_to);
  else if(getIndex(corridor.at_left_, to) >= 0)
    getRightLeft(corridor.at_left_, to, right_to, left_to);

  int from_index, to_index = -1;
  if((to_index = getIndex(corridor.at_begin_edge_, to)) >= 0) //next goal at begin_edge
  {
    std::cout << "NEXT GOAL AT BE" << std::endl;
    getRightLeft(corridor.at_begin_edge_, to, right_to, left_to);
    if((from_index = getIndex(corridor.at_begin_edge_, from)) >= 0)
    {
      if(isBefore(corridor.at_begin_edge_, from, to_index))
        res = getHereSide(from_current, step, nb_steps, to, right);
      else
        res = getHereSide(from_current, step, nb_steps, to, left);
    }
    else if((from_index = getIndex(corridor.at_right_, from)) >= 0)
    {
      res_prelim = sentence_req_t(during_turn_continu_corridor, "", left);
      res = getAtEnd(from_current, step, nb_steps, to);
    }
    else if((from_index = getIndex(corridor.at_left_, from)) >= 0)
    {
      res_prelim = sentence_req_t(during_turn_continu_corridor, "", right);
      res = getAtEnd(from_current, step, nb_steps, to);
    }
    else
      res = getAtEnd(from_current, step, nb_steps, to);
  }
  else if((to_index = getIndex(corridor.at_end_edge_, to)) >= 0) //next goal at end_edge
  {
    std::cout << "NEXT GOAL AT EE" << std::endl;
    getRightLeft(corridor.at_end_edge_, to, right_to, left_to);
    if((from_index = getIndex(corridor.at_end_edge_, from)) >= 0)
    {
      if(isBefore(corridor.at_begin_edge_, from, to_index))
        res = getHereSide(from_current, step, nb_steps, to, right);
      else
        res = getHereSide(from_current, step, nb_steps, to, left);
    }
    else if((from_index = getIndex(corridor.at_right_, from)) >= 0)
    {
      res_prelim = sentence_req_t(during_turn_continu_corridor, "", right);
      res = getAtEnd(from_current, step, nb_steps, to);
    }
    else if((from_index = getIndex(corridor.at_left_, from)) >= 0)
    {
      res_prelim = sentence_req_t(during_turn_continu_corridor, "", left);
      res = getAtEnd(from_current, step, nb_steps, to);
    }
    else
      res = getAtEnd(from_current, step, nb_steps, to);
  }
  else if((to_index = getIndex(corridor.at_right_, to)) >= 0) //next goal at right
  {
    std::cout << "NEXT GOAL AT RIGHT" << std::endl;
    getRightLeft(corridor.at_right_, to, right_to, left_to);
    if((from_index = getIndex(corridor.at_right_, from)) >= 0)
    {
      if(isBefore(corridor.at_right_, from, to_index))
      {
        res_prelim = sentence_req_t(during_turn_continu_corridor, "", right);
        res = getAllSide(from_current, step, nb_steps, to, right);
      }
      else
      {
        res_prelim = sentence_req_t(during_turn_continu_corridor, "", left);
        res = getAllSide(from_current, step, nb_steps, to, left);
      }
    }
    else if((from_index = getIndex(corridor.at_end_edge_, from)) >= 0)
      res = getAllSide(from_current, step, nb_steps, to, left);
    else if((from_index = getIndex(corridor.at_begin_edge_, from)) >= 0)
      res = getAllSide(from_current, step, nb_steps, to, right);
    else // from at left
    {
      if(corridor.in_front_of_.find(from) != corridor.in_front_of_.end())
      {
        if(corridor.in_front_of_[from] == to)
          res = getOsFront(from_current, step, nb_steps, to);
        else if(getIndex(corridor.at_right_, corridor.in_front_of_[from]) > getIndex(corridor.at_right_, to))
        {
          res_prelim = sentence_req_t(during_turn_continu_corridor, "", right); // do not touch
          if(getIndex(corridor.at_right_, to) > 0)
          {
            auto types = onto_->individuals.getUp(corridor.at_right_[getIndex(corridor.at_right_, to) + 1]);
            if(std::find(types.begin(), types.end(), "empty_place") == types.end())
              res = getAllSide(from_current, step, nb_steps, to, left, corridor.at_right_[getIndex(corridor.at_right_, to) + 1]);
            else
              res = getAllSide(from_current, step, nb_steps, to, left);
          }
          else
            res = getAllSide(from_current, step, nb_steps, to, left);
        }
        else
        {
          res_prelim = sentence_req_t(during_turn_continu_corridor, "", left); // do not touch
          if(getIndex(corridor.at_right_, to) < corridor.at_right_.size() - 1)
          {
            auto types = onto_->individuals.getUp(corridor.at_right_[getIndex(corridor.at_right_, to) - 1]);
            if(std::find(types.begin(), types.end(), "empty_place") == types.end())
              res = getAllSide(from_current, step, nb_steps, to, right, corridor.at_right_[getIndex(corridor.at_right_, to) - 1]);
            else
              res = getAllSide(from_current, step, nb_steps, to, right);
          }
          else
            res = getAllSide(from_current, step, nb_steps, to, right);
        }
      }
      else
        getDirectionToRight(res_prelim, res, corridor, from, to, from_current, step, nb_steps);
    }
  }
  else if((to_index = getIndex(corridor.at_left_, to)) >= 0) //next goal at left
  {
    std::cout << "NEXT GOAL AT LEFT" << std::endl;
    getRightLeft(corridor.at_left_, to, right_to, left_to);
    if((from_index = getIndex(corridor.at_left_, from)) >= 0)
    {
      if(isBefore(corridor.at_left_, from, to_index))
      {
        res_prelim = sentence_req_t(during_turn_continu_corridor, "", left);
        res = getAllSide(from_current, step, nb_steps, to, left);
      }
      else
      {
        res_prelim = sentence_req_t(during_turn_continu_corridor, "", right);
        res = getAllSide(from_current, step, nb_steps, to, right);
      }
    }
    else if((from_index = getIndex(corridor.at_end_edge_, from)) >= 0)
      res = getAllSide(from_current, step, nb_steps, to, right);
    else if((from_index = getIndex(corridor.at_begin_edge_, from)) >= 0)
      res = getAllSide(from_current, step, nb_steps, to, left);
    else // from at right
    {
      if(corridor.in_front_of_.find(from) != corridor.in_front_of_.end())
      {
        if(corridor.in_front_of_[from] == to)
          res = getOsFront(from_current, step, nb_steps, to);
        else if(getIndex(corridor.at_left_, corridor.in_front_of_[from]) > getIndex(corridor.at_left_, to))
        {
          res_prelim = sentence_req_t(during_turn_continu_corridor, "", right);
          if(getIndex(corridor.at_left_, to) < corridor.at_left_.size() - 1)
          {
            auto types = onto_->individuals.getUp(corridor.at_left_[getIndex(corridor.at_left_, to) - 1]);
            if(std::find(types.begin(), types.end(), "empty_place") == types.end())
              res = getAllSide(from_current, step, nb_steps, to, left, corridor.at_left_[getIndex(corridor.at_left_, to) - 1]);
            else
              res = getAllSide(from_current, step, nb_steps, to, left);
          }
          else
            res = getAllSide(from_current, step, nb_steps, to, left);
        }
        else
        {
          res_prelim = sentence_req_t(during_turn_continu_corridor, "", left);
          if(getIndex(corridor.at_left_, to) > 0)
          {
            auto types = onto_->individuals.getUp(corridor.at_left_[getIndex(corridor.at_left_, to) + 1]);
            if(std::find(types.begin(), types.end(), "empty_place") == types.end())
              res = getAllSide(from_current, step, nb_steps, to, right, corridor.at_left_[getIndex(corridor.at_left_, to) + 1]);
            else
              res = getAllSide(from_current, step, nb_steps, to, right);
          }
          else
            res = getAllSide(from_current, step, nb_steps, to, right);
        }
      }
      else
        getDirectionToLeft(res_prelim, res, corridor, from, to, from_current, step, nb_steps);
    }
  }
  if(res.reference_ == "")
    setReference(res);

  if(res_prelim.reference_ == "")
    setReference(res_prelim);

  std::vector<sentence_req_t> result;
  if(res_prelim.type_ != none_type)
    result.push_back(res_prelim);
  result.push_back(res);
  return result;
}

void PlaceVerbalizer::getDirectionToLeft(sentence_req_t& res_prelim, sentence_req_t& res, corridor_t& corridor, std::string from, std::string to, bool from_current, size_t step, size_t nb_steps)
{
  size_t from_index = getIndex(corridor.at_right_, from);
  int right_side_index = -1;
  int left_side_index = -1;

  for(int i = from_index-1; i >= 0; i--)
  {
    if(corridor.in_front_of_.find(corridor.at_right_[i]) != corridor.in_front_of_.end())
    {
      left_side_index = i;
      break;
    }
  }

  for(size_t i = from_index+1; i < corridor.at_right_.size(); i++)
  {
    if(corridor.in_front_of_.find(corridor.at_right_[i]) != corridor.in_front_of_.end())
    {
      right_side_index = i;
      break;
    }
  }

  bool found = false;

  if(right_side_index != -1)
  {
    size_t right_other_side_index = getIndex(corridor.at_left_, corridor.in_front_of_[corridor.at_right_[right_side_index]]);
    for(int i = right_other_side_index; i < (int)corridor.at_left_.size(); i++)
      if(corridor.at_left_[i] == to)
      {
        res_prelim = sentence_req_t(during_turn_continu_corridor, "", right);
        res = getAllSide(from_current, step, nb_steps, to, left);
        found = true;
        break;
      }
  }

  if(left_side_index != -1)
  {
    size_t left_other_side_index = getIndex(corridor.at_left_, corridor.in_front_of_[corridor.at_right_[left_side_index]]);
    for(int i = left_other_side_index; i >= 0; i--)
      if(corridor.at_left_[i] == to)
      {
        res_prelim = sentence_req_t(during_turn_continu_corridor, "", left);
        res = getAllSide(from_current, step, nb_steps, to, right);
        found = true;
        break;
      }
  }

  if(found == false)
    std::cout << "Your programmer is bugging ..." << std::endl; //TODO
}

void PlaceVerbalizer::getDirectionToRight(sentence_req_t& res_prelim, sentence_req_t& res, corridor_t& corridor, std::string from, std::string to, bool from_current, size_t step, size_t nb_steps)
{
  size_t from_index = getIndex(corridor.at_left_, from);
  int right_side_index = -1;
  int left_side_index = -1;

  for(int i = from_index-1; i >= 0; i--)
  {
    std::cout << "left test : " << corridor.at_left_[i] << std::endl;
    if(corridor.in_front_of_.find(corridor.at_left_[i]) != corridor.in_front_of_.end())
    {
      left_side_index = i;
      break;
    }
  }

  for(size_t i = from_index+1; i < corridor.at_left_.size(); i++)
  {
    std::cout << "right test : " << corridor.at_left_[i] << std::endl;
    if(corridor.in_front_of_.find(corridor.at_left_[i]) != corridor.in_front_of_.end())
    {
      right_side_index = i;
      break;
    }
  }

  bool found = false;

  if(right_side_index != -1)
  {
    size_t right_other_side_index = getIndex(corridor.at_right_, corridor.in_front_of_[corridor.at_left_[right_side_index]]);
    for(int i = right_other_side_index; i < (int)corridor.at_right_.size(); i++)
      if(corridor.at_right_[i] == to)
      {
        res_prelim = sentence_req_t(during_turn_continu_corridor, "", left);
        res = getAllSide(from_current, step, nb_steps, to, right);
        found = true;
        break;
      }
  }

  if(left_side_index != -1)
  {
    size_t left_other_side_index = getIndex(corridor.at_right_, corridor.in_front_of_[corridor.at_left_[left_side_index]]);
    for(int i = left_other_side_index; i >= 0; i--)
      if(corridor.at_right_[i] == to)
      {
        res_prelim = sentence_req_t(during_turn_continu_corridor, "", right);
        res = getAllSide(from_current, step, nb_steps, to, left);
        found = true;
        break;
      }
  }

  if(found == false)
    std::cout << "Your programmer is bugging ..." << std::endl; //TODO
}

std::vector<sentence_req_t> PlaceVerbalizer::getDirectionOpenspace(std::string& from, std::string& openspace_name, std::string& to, bool from_current, size_t step, size_t nb_steps)
{
  std::vector<sentence_req_t> res;
  openspace_t openspace;

  for(openspace_t& os : openspaces_)
    if(os.name_ == openspace_name)
    {
      openspace = os;
      break;
    }

  if(openspace.name_ != "")
    res = getDirectionOpenspace(from, openspace, to, from_current, step, nb_steps);

  return res;
}

std::vector<sentence_req_t> PlaceVerbalizer::getDirectionOpenspace(std::string& from, openspace_t& openspace, std::string& to, bool from_current, size_t step, size_t nb_steps)
{
  sentence_req_t res;
  int from_index = getIndex(openspace.around_, from);
  if(from_index >= 0)
  {
    right_from = "";
    left_from = "";
    right_to = "";
    left_to = "";

    if(getIndex(openspace.around_, from) >= 0)
      getRightLeftCircle(openspace.around_, from, right_from, left_from);

    if(getIndex(openspace.around_, to) >= 0)
      getRightLeftCircle(openspace.around_, to, right_to, left_to);

    if(openspace.in_front_of_.find(openspace.around_[from_index]) != openspace.in_front_of_.end())
    {
      if(openspace.in_front_of_[openspace.around_[from_index]] == to)
        res = getOsFront(from_current, step, nb_steps, to);
      else
      {
        for(size_t i = 0; i < openspace.around_.size(); i++)
        {
          if(openspace.around_[(from_index + i) % openspace.around_.size()] == to)
          {
            res = getAllSide(from_current, step, nb_steps, to, left);
            break;
          }
          else if(openspace.around_[(from_index + i) % openspace.around_.size()] == openspace.in_front_of_[openspace.around_[from_index]])
          {
            res = getAllSide(from_current, step, nb_steps, to, right);
            break;
          }
        }
      }
    }
    else if(openspace.around_[(from_index + 1) % openspace.around_.size()] == to)
      res = getAllSide(from_current, step, nb_steps, to, left);
    else
    {
      size_t index = from_index - 1;
      if(from_index == 0)
        index = openspace.around_.size() - 1;
      if(openspace.around_[index] == to)
        res = getAllSide(from_current, step, nb_steps, to, right);
      else
      {
        int to_index = getIndex(openspace.around_, to);
        if(to_index >= 0)
          res = getOsRef(from_current, step, nb_steps, to, openspace.around_[(to_index + 1) % openspace.around_.size()], right);
      }
    }
  }
  setReference(res);

  std::vector<sentence_req_t> result;
  result.push_back(res);
  return result;
}

std::vector<sentence_req_t> PlaceVerbalizer::getDirectionRegion(std::string& from, std::string& region_name, std::string& to, bool from_current, size_t step, size_t nb_steps)
{
  std::vector<sentence_req_t> res;

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

sentences_type PlaceVerbalizer::chooseMoment(sentences_type start, sentences_type during, sentences_type end, bool from_current, size_t step, size_t nb_steps)
{
  if(step == nb_steps)
    return end;
  else if((step == 1) && (from_current == true))
    return start;
  else
    return during;
}

sentence_req_t PlaceVerbalizer::getOsFront(bool from_current, size_t step, size_t nb_steps, std::string to)
{
  sentence_req_t res(to);
  std::vector<std::string> to_type = onto_->individuals.getUp(to);
  if(std::find(to_type.begin(), to_type.end(), "interface") != to_type.end())
    res.type_ = chooseMoment(start_interface, during_interface_font, end_in_front, from_current, step, nb_steps);
  else
    res.type_ = chooseMoment(start_corridor, during_interface_font, end_in_front, from_current, step, nb_steps);

  return res;
}

sentence_req_t PlaceVerbalizer::getAllSide(bool from_current, size_t step, size_t nb_steps, std::string to, side_t side, std::string ref)
{
  sentence_req_t res(to, side, ref);
  std::vector<std::string> to_type = onto_->individuals.getUp(to);
  if(std::find(to_type.begin(), to_type.end(), "interface") != to_type.end())
    res.type_ = chooseMoment(start_interface, during_interface_side, end_side, from_current, step, nb_steps);
  else
    res.type_ = chooseMoment(start_corridor, during_turn, end_side, from_current, step, nb_steps);

  return res;
}

sentence_req_t PlaceVerbalizer::getOsRef(bool from_current, size_t step, size_t nb_steps, std::string to, std::string ref, side_t side)
{
  sentence_req_t res(to, none_side, ref, side);
  std::vector<std::string> to_type = onto_->individuals.getUp(to);
  if(std::find(to_type.begin(), to_type.end(), "interface") != to_type.end())
    res.type_ = chooseMoment(start_interface, during_interface_side, end_here, from_current, step, nb_steps);
  else
    res.type_ = chooseMoment(start_corridor, during_interface_side, end_here, from_current, step, nb_steps);

  return res;
}

sentence_req_t PlaceVerbalizer::getHereSide(bool from_current, size_t step, size_t nb_steps, std::string to, side_t side)
{
  sentence_req_t res(to, side);
  std::vector<std::string> to_type = onto_->individuals.getUp(to);
  if(std::find(to_type.begin(), to_type.end(), "interface") != to_type.end())
    res.type_ = chooseMoment(start_interface, during_interface_side, end_here, from_current, step, nb_steps);
  else
    res.type_ = chooseMoment(start_corridor, during_turn, end_here, from_current, step, nb_steps);

  return res;
}

sentence_req_t PlaceVerbalizer::getAtEnd(bool from_current, size_t step, size_t nb_steps, std::string to)
{
  sentence_req_t res(to);
  std::vector<std::string> to_type = onto_->individuals.getUp(to);
  if(std::find(to_type.begin(), to_type.end(), "interface") != to_type.end())
    res.type_ = chooseMoment(start_interface, during_end_of_corridor, end_in_front, from_current, step, nb_steps);
  else
    res.type_ = chooseMoment(start_end_of_corridor, during_end_of_corridor, end_in_front, from_current, step, nb_steps);

  return res;
}

void PlaceVerbalizer::getRightLeft(std::vector<std::string> places, std::string place, std::string& right, std::string& left)
{
  size_t index = getIndex(places, place);
  if(index > 0)
  {
    left = places[index - 1];
    auto types = onto_->individuals.getUp(left);
    if(std::find(types.begin(), types.end(), "empty_place") != types.end())
      left = "";
  }
  if(index < places.size() - 1)
  {
    right = places[index + 1];
    auto types = onto_->individuals.getUp(right);
    if(std::find(types.begin(), types.end(), "empty_place") != types.end())
      right = "";
  }
}

void PlaceVerbalizer::getRightLeftCircle(std::vector<std::string> places, std::string place, std::string& right, std::string& left)
{
  size_t index = getIndex(places, place);
  if(index > 0)
    right = places[index - 1];
  else
    right = places[places.size() - index - 1];
  if(index < places.size() - 1)
    left = places[index + 1];
  else
    left = places[(index + 1) % places.size()];
}

void PlaceVerbalizer::setReference(sentence_req_t& req)
{
  if((req.type_ == during_turn_continu_corridor) ||
     (req.type_ == during_turn) ||
     (req.type_ == during_interface_font) ||
     (req.type_ == during_interface_side))
    setReference(req, right_from, left_from);
  else
    setReference(req, right_to, left_to);
}

void PlaceVerbalizer::setReference(sentence_req_t& req, std::string right_place, std::string left_place)
{
  auto types = onto_->individuals.getUp(right_place);
  if(std::find(types.begin(), types.end(), "empty_place") != types.end())
    right_place = "";
  types = onto_->individuals.getUp(left_place);
  if(std::find(types.begin(), types.end(), "empty_place") != types.end())
    left_place = "";

  if(req.type_ == end_side)
  {
    std::cout << "reference: is an end side" << std::endl;
    if(req.side_ == right)
      std::cout << "--> req at right => left place = " << right_place << std::endl;
    else
      std::cout << "--> req at left => right place = " << left_place << std::endl;
    if((req.side_ == right) && (right_place != ""))
    {
      req.reference_ = right_place;
      req.refrence_side_ = left;
    }
    else if((req.side_ == left) && (left_place != ""))
    {
      req.reference_ = left_place;
      req.refrence_side_ = right;
    }
  }
  else if(right_place == "")
  {
    std::cout << "right empty" << std::endl;
    if(left_place != "")
    {
      std::cout << "left full" << std::endl;
      req.reference_ = left_place;
      req.refrence_side_ = left;
    }
  }
  else if(left_place == "")
  {
    std::cout << "left empty" << std::endl;
    if(right_place != "")
    {
      std::cout << "right full" << std::endl;
      req.reference_ = right_place;
      req.refrence_side_ = right;
    }
  }
  else
  {
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<> dis(0, 1);
    size_t index = dis(gen);

    if(index == 0)
    {
      req.reference_ = left_place;
      req.refrence_side_ = left;
    }
    else
    {
      req.reference_ = right_place;
      req.refrence_side_ = right;
    }
  }
  std::cout << "setReference = " << req.reference_ << " at " << req.refrence_side_ << std::endl;
  std::cout << right_place << " <=> " << left_place << std::endl;
}

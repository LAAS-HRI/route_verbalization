#ifndef PLACEVERBALIZER_H
#define PLACEVERBALIZER_H

#include "ontoloGenius/utility/OntologyManipulator.h"
#include "route_verbalization/mapManipulators/MapReader.h"
#include "route_verbalization/Verbalizers/sentences.h"

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
  Sentences sentences;

  std::vector<corridor_t> corridors_;
  std::vector<openspace_t> openspaces_;
  std::map<std::string, std::string> word_map;

  std::string right_from, left_from, right_to, left_to;

  std::vector<sentence_req_t> getDirection(std::string& from, std::string& path, std::string& to, bool from_current, size_t step, size_t nb_steps);
  std::vector<sentence_req_t> getDirectionPath(std::string& from, std::string& path, std::string& to, bool from_current, size_t step, size_t nb_steps);
  std::vector<sentence_req_t> getDirectionCorridor(std::string& from, std::string& corridor_name, std::string& to, bool from_current, size_t step, size_t nb_steps);
  std::vector<sentence_req_t> getDirectionCorridor(std::string& from, corridor_t& corridor, std::string& to, bool from_current, size_t step, size_t nb_steps);
  std::vector<sentence_req_t> getDirectionOpenspace(std::string& from, std::string& openspace_name, std::string& to, bool from_current, size_t step, size_t nb_steps);
  std::vector<sentence_req_t> getDirectionOpenspace(std::string& from, openspace_t& openspace, std::string& to, bool from_current, size_t step, size_t nb_steps);
  std::vector<sentence_req_t> getDirectionRegion(std::string& from, std::string& region_name, std::string& to, bool from_current, size_t step, size_t nb_steps);
  void getDirectionToLeft(sentence_req_t& res_prelim, sentence_req_t& res, corridor_t& corridor, std::string from, std::string to, bool from_current, size_t step, size_t nb_steps);
  void getDirectionToRight(sentence_req_t& res_prelim, sentence_req_t& res, corridor_t& corridor, std::string from, std::string to, bool from_current, size_t step, size_t nb_steps);

  int getIndex(std::vector<std::string>& vect, std::string& word);
  bool isBefore(std::vector<std::string>& vect, std::string& word, size_t index);
  sentences_type chooseMoment(sentences_type start, sentences_type during, sentences_type end, bool from_current, size_t step, size_t nb_steps);

  sentence_req_t getOsFront(bool from_current, size_t step, size_t nb_steps, std::string to);
  sentence_req_t getAllSide(bool from_current, size_t step, size_t nb_steps, std::string to, side_t side, std::string ref = "");
  sentence_req_t getOsRef(bool from_current, size_t step, size_t nb_steps, std::string to, std::string ref, side_t side);
  sentence_req_t getHereSide(bool from_current, size_t step, size_t nb_steps, std::string to, side_t side);
  sentence_req_t getAtEnd(bool from_current, size_t step, size_t nb_steps, std::string to);

  void getRightLeft(std::vector<std::string> places, std::string place, std::string& right, std::string& left);
  void getRightLeftCircle(std::vector<std::string> places, std::string place, std::string& right, std::string& left);
  void setReference(sentence_req_t& req);
  void setReference(sentence_req_t& req, std::string right_place, std::string left_place);
};

#endif

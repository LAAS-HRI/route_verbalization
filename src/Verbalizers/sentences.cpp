#include "route_verbalization/Verbalizers/sentences.h"
#include <random>

Sentences::Sentences(OntologyManipulator* onto) : onto_(onto)
{
  createEnd();
  createBegin();
  createDuring();

  interface_map["elevator"] = "take the ";
  interface_map["stair"] = "take the ";
  interface_map["escalator"] = "take the ";
}

void Sentences::createEnd()
{
  {
    sentence_t tmp(end_side,
      {{"and ", "then ", ", "}, {"you'll "}, {"see ", "find "},
      {"it ", "/X "}, {"on "}, {"your ", "the "}, {"/D "},
      {"side ", "when you walk ", ""}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{"and ", "then ", "it's then ", "then ", ", "}, {"on "}, {"the ", "your "},
      {"/D "}, {"side ", "when you walk", ""}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{"then ", ", "}, {"it will be on your "}, {"/D "}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{"then ", ", "}, {"it's on the "}, {"/D "}, {"there "}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_side,
      {{"and ", "then ", ", "}, {"on the "}, {"/D "}, {"you'll see "}, {"/X "}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{"and ", "then ", ", "}, {"you see there "}, {"/X "}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{"and ", "then ", ", "}, {"you'll find it there "}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_here,
      {{"and ", "then ", ", "}, {"it's "}, {"there ", ""}, {"on the "}, {"/D "}, {"side of "}, {"/Y "}});
    end_.push_back(tmp);
  }

  {
    sentence_t tmp(end_in_front,
      {{"and ", "then ", ", "}, {"you'll "}, {"find ", "see "}, {"it ", "/X "}, {"right away "}});
    end_.push_back(tmp);
  }
}

void Sentences::createBegin()
{
  {
    sentence_t tmp(start_corridor,
      {{"just ", "you "}, {"go ", "walk ", "go straight "},
      {"across ", "through ", "on ", "down "}, {"that ", "this "}, {"corridor ", "aisle "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_corridor,
      {{"you go to this "}, {"corridor ", "aisle "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_end_of_corridor,
      {{"/X "}, {"is "}, {"just straight ", ""}, {"down "}, {"this ", "that "}, {"corridor ", "aisle "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_end_of_corridor,
      {{"just ", ""}, {"walk ", "go almost "}, {"until the end of "}, {"this ", "that "}, {"corridor ", "aisle "}});
    begin_.push_back(tmp);
  }

  {
    sentence_t tmp(start_end_of_corridor,
      {{"/X "}, {"is at the end of "}, {"this ", "that "}, {"corridor ", "aisle "}});
    begin_.push_back(tmp);
  }
}

void Sentences::createDuring()
{
  {
    sentence_t tmp(during_end_of_corridor,
      {{"you walk to ", "almost at ", "walk until "}, {"the "}, {"very ", ""}, {"end "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_turn_continu_corridor,
      {{"turn "}, {"/D "}, {"at ", "straight after "}, {"/Y "}, {"and "}, {"continue straight ", "walk down the corridor "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_turn,
      {{"turn "}, {"/D "}, {"end you will see /Y ", ""}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_turn,
      {{"you'll "}, {"see ", "find "}, {"/Y "}, {"turning right to the "}, {"/D "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_reference,
      {{"you'll "}, {"see ", "find "}, {"/Y "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_interface_font,
      {{"/I "}, {"just in front ", "right away "}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_interface_side,
      {{"/I "}, {"at "}, {"the ", "your "}, {"/D "}, {"side ", ""}});
    during_.push_back(tmp);
  }

  {
    sentence_t tmp(during_interface_side,
      {{"/I "}, {"that it's "}, {"ont he /DY side of ", "next to ", "at /DY of "}, {"/Y "}});
    during_.push_back(tmp);
  }
}

std::string Sentences::createInterfaceSentence(std::string word)
{
  std::vector<std::string> interfaces = onto_->individuals.getUp(word, 1);
  std::string interface_name = "";
  std::string verb = "go through the ";
  for(size_t interface = 0; interface < interfaces.size(); interface++)
      if (interface_map.find(interfaces[interface]) != interface_map.end())
      {
        verb = interface_map[interfaces[interface]];
        interface_name = interfaces[interface];
      }

  if(interface_name == "")
    interface_name = onto_->individuals.getName(word);

  return verb + interface_name + " ";
}

std::string Sentences::getSentence(sentence_req_t req)
{
  if((req.type_ == end_side) || (req.type_ == end_here) || (req.type_ == end_in_front))
    return getSentence(req, end_);
  else if((req.type_ == start_corridor) || (req.type_ == start_end_of_corridor) || (req.type_ == start_interface))
    return getSentence(req, begin_);
  else
    return getSentence(req, during_);
}

std::string Sentences::getSentence(sentence_req_t& req, std::vector<sentence_t>& base)
{
  std::string res;
  size_t timeout = 10;
  do
  {
    std::vector<sentence_t> sentences_base;
    getSentences(sentences_base, base, req.type_);
    res = selectOne(sentences_base);
    timeout--;
  }
  while((replace(res, req) == false) && (timeout > 0));

  return res;
}

void Sentences::getSentences(std::vector<sentence_t>& sentences, std::vector<sentence_t>& base, sentences_type type)
{
  for(size_t i = 0; i < base.size(); i++)
    if(base[i].type_ == type)
      sentences.push_back(base[i]);
}

std::string Sentences::selectOne(std::vector<sentence_t>& sentences)
{
  std::random_device rd;
  std::mt19937 gen(rd());

  std::vector<size_t> costs;
  size_t total = 0;
  for(size_t i = 0; i < sentences.size(); i++)
  {
    total += sentences[i].getNb();
    costs.push_back(total);
  }
  std::uniform_int_distribution<> global_dis(1, total);
  size_t sentence_index = global_dis(gen);
  sentence_t selected;

  for(size_t i = 0; i < sentences.size(); i++)
    if(sentence_index <= costs[i])
    {
      selected = sentences[i];
      break;
    }

  std::string res;
  for(size_t i = 0; i < selected.text_.size(); i++)
  {
    std::uniform_int_distribution<> dis(0, selected.text_[i].size() - 1);
    size_t index = dis(gen);
    res += selected.text_[i][index];
  }
  return res;
}

bool Sentences::replace(std::string& text, sentence_req_t& req)
{
  size_t pose = std::string::npos;
  while((pose = text.find("/D")) != std::string::npos)
  {
    if(req.side_ != none_side)
      if(req.side_ != right)
        text.replace(pose, pose + 2, "right");
      else
        text.replace(pose, pose + 2, "left");
    else
      return false;
  }

  while((pose = text.find("/DY")) != std::string::npos)
  {
    if(req.refrence_side_ != none_side)
      if(req.refrence_side_ != right)
        text.replace(pose, pose + 3, "right");
      else
        text.replace(pose, pose + 3, "left");
    else
      return false;
  }

  while((pose = text.find("/X")) != std::string::npos)
  {
    if(req.place_ != "")
    {
      std::string name = onto_->individuals.getName(req.place_);
      text.replace(pose, pose + 2, name);
    }
    else
      return false;
  }

  while((pose = text.find("/Y")) != std::string::npos)
  {
    if(req.reference_ != "")
    {
      std::string name = onto_->individuals.getName(req.reference_);
      text.replace(pose, pose + 2, name);
    }
    else
      return false;
  }

  return true;
}

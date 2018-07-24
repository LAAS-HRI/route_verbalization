#ifndef SENTENCES_H
#define SENTENCES_H

#include <string>
#include <vector>
#include "ontoloGenius/utility/OntologyManipulator.h"

enum sentences_type
{
  end_side,
  end_here,
  end_in_front,
  start_corridor,
  start_end_of_corridor,
  start_interface,
  during_end_of_corridor,
  during_turn_continu_corridor,
  during_turn,
  during_reference,
  during_interface_font,
  during_interface_side,
  none_type
};

enum side_t
{
  right,
  left,
  none_side
};

struct sentence_t
{
  std::vector<std::vector<std::string> > text_;
  sentences_type type_;
  size_t getNb()
  {
    if(text_.size())
    {
      size_t nb = text_[0].size();
      for(size_t i = 1; i < text_.size(); i++)
        nb = nb * text_[i].size();
      return nb;
    }
    else
      return 0;
  }

  sentence_t(sentences_type type, std::vector<std::vector<std::string> > text)
  {
    type_ = type;
    text_ = text;
  }
  sentence_t()
  {
    type_ = none_type;
  }
};

struct sentence_req_t
{
  sentences_type type_;
  std::string place_;
  side_t side_;
  std::string reference_;
  side_t refrence_side_;

  sentence_req_t(sentences_type type, std::string place, side_t side = none_side, std::string reference = "", side_t refrence_side = none_side)
  {
    type_ = type;
    place_ = place;
    side_ = side;
    reference_ = reference;
    refrence_side_ = refrence_side;
  }
  sentence_req_t(std::string place, side_t side = none_side, std::string reference = "", side_t refrence_side = none_side)
  {
    type_ = none_type;
    place_ = place;
    side_ = side;
    reference_ = reference;
    refrence_side_ = refrence_side;
  }
  sentence_req_t() {type_ = none_type;}
};

class Sentences
{
public:
  Sentences(OntologyManipulator* onto);

  std::string getSentence(sentence_req_t req);

private:
  OntologyManipulator* onto_;
  std::vector<sentence_t> end_;
  std::vector<sentence_t> begin_;
  std::vector<sentence_t> during_;
  std::map<std::string, std::string> interface_map;

  void createEnd();
  void createBegin();
  void createDuring();

  std::string createInterfaceSentence(std::string word);

  std::string getSentence(sentence_req_t& req, std::vector<sentence_t>& base);
  void getSentences(std::vector<sentence_t>& sentences, std::vector<sentence_t>& base, sentences_type type);
  std::string selectOne(std::vector<sentence_t>& sentences);
  bool replace(std::string& text, sentence_req_t& req);
};

#endif

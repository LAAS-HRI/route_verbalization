#include "route_verbalization/OntologyManipulator.h"

#include "ontologenius/OntologeniusService.h"

std::vector<std::string> OntologyManipulator::string2vector(const std::string& value)
{
  std::vector<std::string> result;
  bool eof = false;
  size_t begin = 0;
  do
  {
    size_t stop = value.find(" ", begin);
    std::string name = value.substr(begin , stop - begin);
    if(stop == std::string::npos)
      eof = true;
    else
      result.push_back(name);
    begin = stop + 1;
  }
  while(eof == false);

  return result;
}

std::vector<std::string> OntologyManipulator::getOn(const std::string& name, const std::string& property)
{
  std::vector<std::string> res;
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/individual");

  ontologenius::OntologeniusService srv;
  srv.request.action = "getOn";
  srv.request.param = name + ":" + property;

  cpt++;

  if(client.call(srv))
    return srv.response.values;
  else
    return res;
}

std::vector<std::string> OntologyManipulator::getFrom(const std::string& property, const std::string& name, const std::string& selector)
{
  std::vector<std::string> res;
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/individual");

  ontologenius::OntologeniusService srv;
  if(selector == "")
  {
    srv.request.action = "getFrom";
    srv.request.param = name + ":" + property;
  }
  else
  {
    srv.request.action = "select:getFrom";
    srv.request.param = selector + "=" + name + ":" + property;
  }

  cpt++;

  if(client.call(srv))
    return srv.response.values;
  else
    return res;
}

std::vector<std::string> OntologyManipulator::getWith(const std::string& indiv_1, const std::string& indiv_2, const std::string& selector)
{
  std::vector<std::string> res;
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/individual");

  ontologenius::OntologeniusService srv;
  if(selector == "")
  {
    srv.request.action = "getWith";
    srv.request.param = indiv_1 + ":" + indiv_2;
  }
  else
  {
    srv.request.action = "select:getWith";
    srv.request.param = selector + "=" + indiv_1 + ":" + indiv_2;
  }

  cpt++;

  if(client.call(srv))
    return srv.response.values;
  else
    return res;
}

std::vector<std::string> OntologyManipulator::getUp(std::string& name, int depth, const std::string& selector)
{
  std::vector<std::string> res;
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/individual");

  ontologenius::OntologeniusService srv;
  if(selector == "")
  {
    srv.request.action = "getUp";
    srv.request.param = name;
  }
  else
  {
    srv.request.action = "select:getUp";
    srv.request.param = selector + "=" + name;
  }

  if(depth >= 0)
    srv.request.param += " < " + std::to_string(depth);

  cpt++;

  if(client.call(srv))
    return srv.response.values;
  else
    return res;
}

bool OntologyManipulator::isA(std::string& name, const std::string& base_class)
{
  std::vector<std::string> res = getUp(name, -1, base_class);
  if(res.size() == 0)
    return false;
  else
    return true;
}

std::vector<std::string> OntologyManipulator::getDown(std::string& name)
{
  std::vector<std::string> res;
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/individual");

  ontologenius::OntologeniusService srv;
  srv.request.action = "getDown";
  srv.request.param = name;

  cpt++;

  if(client.call(srv))
    return srv.response.values;
  else
    return res;
}

std::vector<std::string> OntologyManipulator::getRelatedFrom(const std::string& name)
{
  std::vector<std::string> res;
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/individual");

  ontologenius::OntologeniusService srv;
  srv.request.action = "getRelatedFrom";
  srv.request.param = name;

  cpt++;

  if(client.call(srv))
    return srv.response.values;
  else
    return res;
}

std::vector<std::string> OntologyManipulator::getType(const std::string& name)
{
  std::vector<std::string> res;
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/individual");

  ontologenius::OntologeniusService srv;
  srv.request.action = "getType";
  srv.request.param = name;

  cpt++;

  if(client.call(srv))
    return srv.response.values;
  else
    return res;
}

std::string OntologyManipulator::getName(const std::string& name)
{
  std::string res = "";
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/individual");

  ontologenius::OntologeniusService srv;
  srv.request.action = "getName";
  srv.request.param = name;

  cpt++;

  if(client.call(srv))
  {
    if(srv.response.values.size())
      return srv.response.values[0];
    else
      return res;
  }
  else
    return res;
}

bool OntologyManipulator::close()
{
  ros::ServiceClient client = n_->serviceClient<ontologenius::OntologeniusService>("ontologenius/actions");
  ontologenius::OntologeniusService srv;
  srv.request.action = "close";

  cpt++;

  if(!client.call(srv))
    return false;
  else
    return true;
}

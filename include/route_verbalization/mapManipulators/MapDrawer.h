#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "route_verbalization/mapManipulators/MapReader.h"

#include <cv.h>
#include <highgui.h>
#include <opencv2/highgui/highgui_c.h>

struct rect_t
{
  long int x,y;
  long int width, height;
  bool fix_;

  rect_t(long int p_x = 0, long int p_y = 0, long int p_width = 0, long int p_height = 0) : x(p_x), y(p_y), width(p_width), height(p_height) {fix_ = false; }

  long int x_top() { return x - width/2; }
  long int y_top() { return y - height/2; }
  long int x_bot() { return x + width/2; }
  long int y_bot() { return y + height/2; }

  static const int WIDTH = 100;
  static const int HEIGHT = 50;
};

struct drawCorridor_t
{
  std::string name_;

  std::vector<std::string> at_right_names_;
  std::vector<std::string> at_left_names_;
  std::vector<std::string> at_begin_edge_names_;
  std::vector<std::string> at_end_edge_names_;

  std::vector<rect_t> at_right_rects_;
  std::vector<rect_t> at_left_rects_;
  std::vector<rect_t> at_begin_edge_rects_;
  std::vector<rect_t> at_end_edge_rects_;
};

class MapDrawer
{
public:
  MapDrawer() {}
  ~MapDrawer(){}

  void draw(std::vector<corridor_t> corridors);
  void draw(std::vector<openspace_t> openspaces);

private:
  onto::OntologyManipulator onto_;

  IplImage* image;
  std::vector<drawCorridor_t> corridors_;

  corridor_t openspace2corridor(openspace_t openspace);

  void drawOneCorridor(corridor_t corridor);
  void set_rect(rect_t rect, std::string name);

  void findRightLeftTrigger(corridor_t corridor, int& right_trig, int& left_trig);
  int getPreviousFix(std::vector<rect_t> rects, size_t pose);

  cv::Scalar getColor(std::string name);
};

#endif

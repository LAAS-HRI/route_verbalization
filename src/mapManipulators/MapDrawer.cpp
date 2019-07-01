#include "route_verbalization/mapManipulators/MapDrawer.h"
#include <opencv2/imgproc/imgproc.hpp>

void MapDrawer::draw(std::vector<corridor_t> corridors)
{
  for(size_t i = 0; i < corridors.size(); i++)
    drawOneCorridor(corridors[i]);
}

void MapDrawer::draw(std::vector<openspace_t> openspaces)
{
  for(size_t i = 0; i < openspaces.size(); i++)
    drawOneCorridor(openspace2corridor(openspaces[i]));
}

corridor_t MapDrawer::openspace2corridor(openspace_t openspace)
{
  corridor_t res;
  size_t half = openspace.around_.size() / 2;
  std::vector<std::string> side_I, side_II;

  for(size_t i = 0; i < half; i++)
    side_I.push_back(openspace.around_[i]);
  for(size_t i = half; i < openspace.around_.size(); i++)
    side_II.push_back(openspace.around_[i]);

  std::vector<std::string> side_1, side_2, side_3, side_4;
  half = side_I.size() / 2;
  for(size_t i = 0; i < half; i++)
    side_1.push_back(side_I[i]);
  for(size_t i = half; i < side_I.size(); i++)
    side_2.push_back(side_I[i]);
  half = side_II.size() / 2;
  for(size_t i = 0; i < half; i++)
    side_3.push_back(side_II[i]);
  for(size_t i = half; i < side_II.size(); i++)
    side_4.push_back(side_II[i]);

  res.name_ = openspace.name_;
  res.in_front_of_ = openspace.in_front_of_;
  res.at_end_edge_ = side_1;
  std::reverse(side_2.begin(),side_2.end());
  res.at_right_ = side_2;
  std::reverse(side_3.begin(),side_3.end());
  res.at_begin_edge_ = side_3;
  res.at_left_ = side_4;

  return res;
}

void MapDrawer::drawOneCorridor(corridor_t corridor)
{
  size_t nb_places = 0;

  image = cvCreateImage(cvSize(3000, 1000), IPL_DEPTH_8U, 3);
  cvSet(image, cvScalar(255,255,255));

  drawCorridor_t to_draw;
  size_t height_offset = 50;
  size_t width_offset = 50;

  for(size_t begin_i = 0; begin_i < corridor.at_begin_edge_.size(); begin_i++)
  {
    rect_t rect(width_offset, height_offset + rect_t::HEIGHT * (begin_i + 1), rect_t::WIDTH, rect_t::HEIGHT);
    to_draw.at_begin_edge_rects_.push_back(rect);
    to_draw.at_begin_edge_names_.push_back(corridor.at_begin_edge_[begin_i]);
    nb_places++;
  }

  for(size_t left_i = 0; left_i < corridor.at_left_.size(); left_i++)
  {
    rect_t rect(width_offset + rect_t::WIDTH * (left_i + 1), height_offset, rect_t::WIDTH, rect_t::HEIGHT);
    to_draw.at_left_rects_.push_back(rect);
    to_draw.at_left_names_.push_back(corridor.at_left_[left_i]);
    nb_places++;
  }

  height_offset = height_offset + (corridor.at_begin_edge_.size() + 1) * rect_t::HEIGHT;

  for(size_t right_i = 0; right_i < corridor.at_right_.size(); right_i++)
  {
    rect_t rect(width_offset + rect_t::WIDTH * (right_i + 1), height_offset, rect_t::WIDTH, rect_t::HEIGHT);
    to_draw.at_right_rects_.push_back(rect);
    to_draw.at_right_names_.push_back(corridor.at_right_[right_i]);
    nb_places++;
  }

  int right_trig_pose = -1;
  int left_trig_pose = -1;
  findRightLeftTrigger(corridor, right_trig_pose, left_trig_pose);
  if((right_trig_pose != -1) && (left_trig_pose != -1))
  {
    while((right_trig_pose != -1) && (left_trig_pose != -1))
    {
      to_draw.at_right_rects_[right_trig_pose].fix_ = true;
      to_draw.at_left_rects_[left_trig_pose].fix_ = true;

      if(right_trig_pose > left_trig_pose)
      {
        int move_of = to_draw.at_right_rects_[right_trig_pose].x - to_draw.at_left_rects_[left_trig_pose].x;
        for(size_t left_i = left_trig_pose; left_i < to_draw.at_left_rects_.size(); left_i++)
          to_draw.at_left_rects_[left_i].x += move_of;

        int fix = getPreviousFix(to_draw.at_left_rects_, left_trig_pose);
        if(fix != -1)
        {
          int delta = to_draw.at_left_rects_[left_trig_pose].x_top() - to_draw.at_left_rects_[fix].x_bot();
          size_t nb = left_trig_pose - fix + 1;
          int add = delta /nb;

          to_draw.at_left_rects_[fix].x += add/2;
          to_draw.at_left_rects_[fix].width += add;
          for(size_t move_i = 1; move_i < nb - 1; move_i++)
          {
            to_draw.at_left_rects_[fix + move_i].x += add/2 + add*move_i;
            to_draw.at_left_rects_[fix + move_i].width += add;
          }
          to_draw.at_left_rects_[left_trig_pose].x -= add/2;
          to_draw.at_left_rects_[left_trig_pose].width += add;
        }
      }
      else if(right_trig_pose < left_trig_pose)
      {
        int move_of = to_draw.at_left_rects_[left_trig_pose].x - to_draw.at_right_rects_[right_trig_pose].x;
        for(size_t right_i = right_trig_pose; right_i < to_draw.at_right_rects_.size(); right_i++)
          to_draw.at_right_rects_[right_i].x += move_of;

        int fix = getPreviousFix(to_draw.at_right_rects_, right_trig_pose);
        if(fix != -1)
        {
          int delta = to_draw.at_right_rects_[right_trig_pose].x_top() - to_draw.at_right_rects_[fix].x_bot();
          size_t nb = right_trig_pose - fix + 1;
          int add = delta /nb;

          to_draw.at_right_rects_[fix].x += add/2;
          to_draw.at_right_rects_[fix].width += add;
          for(size_t move_i = 1; move_i < nb - 1; move_i++)
          {
            to_draw.at_right_rects_[fix + move_i].x += add/2 + add*move_i;
            to_draw.at_right_rects_[fix + move_i].width += add;
          }
          to_draw.at_right_rects_[right_trig_pose].x -= add/2;
          to_draw.at_right_rects_[right_trig_pose].width += add;
        }
      }
      else
      {
        std::cout << "ALIGNED" << std::endl;
      }

      findRightLeftTrigger(corridor, right_trig_pose, left_trig_pose);
    }
  }

  if((to_draw.at_right_rects_.size() > 0) && (to_draw.at_left_rects_.size() > 0))
    if(to_draw.at_right_rects_[to_draw.at_right_rects_.size() - 1].x_bot() > to_draw.at_left_rects_[to_draw.at_left_rects_.size() - 1].x_bot())
      width_offset = to_draw.at_right_rects_[to_draw.at_right_rects_.size() - 1].x_bot() + rect_t::WIDTH/2;
    else
      width_offset = to_draw.at_left_rects_[to_draw.at_left_rects_.size() - 1].x_bot() + rect_t::WIDTH/2;
  else if(to_draw.at_right_rects_.size() > 0)
    width_offset = to_draw.at_right_rects_[to_draw.at_right_rects_.size() - 1].x_bot() + rect_t::WIDTH/2;
  else if(to_draw.at_left_rects_.size() > 0)
    width_offset = to_draw.at_left_rects_[to_draw.at_left_rects_.size() - 1].x_bot() + rect_t::WIDTH/2;

  for(size_t end_i = 0; end_i < corridor.at_end_edge_.size(); end_i++)
  {
    rect_t rect(width_offset, 50 + rect_t::HEIGHT * (end_i + 1), rect_t::WIDTH, rect_t::HEIGHT);
    to_draw.at_end_edge_rects_.push_back(rect);
    to_draw.at_end_edge_names_.push_back(corridor.at_end_edge_[end_i]);
    nb_places++;
  }

  if((to_draw.at_end_edge_rects_.size() > 0) && (to_draw.at_begin_edge_rects_.size() > 0))
    if(to_draw.at_end_edge_rects_[to_draw.at_end_edge_rects_.size() - 1].y_bot() > to_draw.at_begin_edge_rects_[to_draw.at_begin_edge_rects_.size() - 1].y_bot())
      height_offset = to_draw.at_end_edge_rects_[to_draw.at_end_edge_rects_.size() - 1].y_bot() + rect_t::HEIGHT/2;
    else
      height_offset = to_draw.at_begin_edge_rects_[to_draw.at_begin_edge_rects_.size() - 1].y_bot() + rect_t::HEIGHT/2;
  else if(to_draw.at_end_edge_rects_.size() > 0)
    height_offset = to_draw.at_end_edge_rects_[to_draw.at_end_edge_rects_.size() - 1].y_bot() + rect_t::HEIGHT/2;
  else if(to_draw.at_begin_edge_rects_.size() > 0)
    height_offset = to_draw.at_begin_edge_rects_[to_draw.at_begin_edge_rects_.size() - 1].y_bot() + rect_t::HEIGHT/2;
  for(size_t right_i = 0; right_i < corridor.at_right_.size(); right_i++)
  {
    to_draw.at_right_rects_[right_i].y = height_offset;
  }

  for(size_t begin_i = 0; begin_i < to_draw.at_begin_edge_rects_.size(); begin_i++)
    set_rect(to_draw.at_begin_edge_rects_[begin_i], to_draw.at_begin_edge_names_[begin_i]);

  for(size_t left_i = 0; left_i < to_draw.at_left_rects_.size(); left_i++)
    set_rect(to_draw.at_left_rects_[left_i], to_draw.at_left_names_[left_i]);

  for(size_t right_i = 0; right_i < to_draw.at_right_rects_.size(); right_i++)
    set_rect(to_draw.at_right_rects_[right_i], to_draw.at_right_names_[right_i]);

  for(size_t end_i = 0; end_i < to_draw.at_end_edge_rects_.size(); end_i++)
    set_rect(to_draw.at_end_edge_rects_[end_i], to_draw.at_end_edge_names_[end_i]);

  if(nb_places != 0)
  {
    cvSaveImage(std::string(corridor.name_ + ".png").c_str(), image);
    std::cout << "SAVE : " << corridor.name_ << ".png" << std::endl;
  }
}

void MapDrawer::set_rect(rect_t rect, std::string name)
{
  cv::Scalar color = getColor(name);
  cvRectangle(image, cvPoint(rect.x_top(), rect.y_top()),
            cvPoint(rect.x_bot(), rect.y_bot()),
            color,
            -1, 8, 0);

  cvRectangle(image, cvPoint(rect.x_top(), rect.y_top()),
            cvPoint(rect.x_bot(), rect.y_bot()),
            color - cv::Scalar(50, 50, 50),
            1, 8, 0);

  CvFont font;
  cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.4, 0.6, 0, 1);
  cvPutText(image, name.c_str(), cvPoint(rect.x_top() + 3, rect.y_top() + 20), &font,
            cv::Scalar(0, 0, 0));
}

void MapDrawer::findRightLeftTrigger(corridor_t corridor, int& right_trig, int& left_trig)
{
  int right_trig_pose;
  right_trig >= 0 ? right_trig_pose = right_trig + 1 : right_trig_pose = 0;
  int left_trig_pose = -1;
  left_trig >= 0 ? left_trig_pose = left_trig + 1 : left_trig_pose = 0;

  right_trig = -1;
  left_trig = -1;

  for(size_t right_i = right_trig_pose; right_i < corridor.at_right_.size(); right_i++)
  {
    if(corridor.in_front_of_.find(corridor.at_right_[right_i]) != corridor.in_front_of_.end())
    {
      right_trig = right_i;
      for(size_t left_i = left_trig_pose; left_i < corridor.at_left_.size(); left_i++)
        if(corridor.at_left_[left_i] == corridor.in_front_of_[corridor.at_right_[right_i]])
        {
          left_trig = left_i;
          break;
        }
      break;
    }
  }
}

int MapDrawer::getPreviousFix(std::vector<rect_t> rects, size_t pose)
{
  for(int i = pose - 1; i >= 0; i--)
    if(rects[i].fix_ == true)
      return i;

  return -1;
}

cv::Scalar MapDrawer::getColor(std::string name)
{
  std::vector<std::string> up = onto_.individuals.getUp(name);

  if(std::find(up.begin(), up.end(), "interface") != up.end())
    return cv::Scalar(255, 100, 0);
  else if(std::find(up.begin(), up.end(), "restaurant") != up.end())
    return cv::Scalar(0, 160, 255);
  else if(std::find(up.begin(), up.end(), "clothes_shop") != up.end())
    return cv::Scalar(0, 0, 255);
  else if(std::find(up.begin(), up.end(), "pathIntersection") != up.end())
    return cv::Scalar(60, 170, 60);
  else
    return cv::Scalar(255, 150, 150);
}

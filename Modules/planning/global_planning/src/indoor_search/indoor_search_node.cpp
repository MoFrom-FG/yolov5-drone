#include <ros/ros.h>

#include "indoor_search.h"

using namespace Global_Planning;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "indoor_search");

  ros::NodeHandle nh("~");

  Global_Planner global_planner;
  global_planner.init(nh);

  ros::spin();

  return 0;
}


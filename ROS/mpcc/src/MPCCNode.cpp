#include "MPCC/MPCC.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "mpcc_node");

  MPCC mpcc_node;
  if (mpcc_node.init())
  {
    mpcc_node.run(); 
  }
  else
  {
    ROS_FATAL_STREAM("mpcc_node initialization failed. Shutdown.");
  }

  return 0;
}


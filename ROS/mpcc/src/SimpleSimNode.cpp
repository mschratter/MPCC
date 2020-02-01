#include "Simple_Sim/SimpleSim.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_simulator_node");

  SimpleSim simple_sim_node;
  if (simple_sim_node.init())
  {
    simple_sim_node.run(); 
  }
  else
  {
    ROS_FATAL_STREAM("simple_sim_node initialization failed. Shutdown.");
  }

  return 0;
}


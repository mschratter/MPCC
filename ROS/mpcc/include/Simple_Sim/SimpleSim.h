#ifndef _SIMPLE_SIM_H_
#define _SIMPLE_SIM_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>


#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>



#include <cmath>
#include <vector>


#include <stdlib.h>

#include <sstream>
#include <fstream>
#include <string>




#include "../C++/MPC/mpc.h"
#include "../C++/Model/integrator.h"

#include <mpcc_msgs/Input.h>
#include <mpcc_msgs/State.h>

#include <tf/tf.h>

class SimpleSim {
  
  // Attributes
  bool m_system_ready;
  bool m_node_initialized;
  double m_rate;


  mpcc::Integrator m_integrator;
  mpcc::State m_x;
  mpcc::Input m_u;

  mpcc::Param m_model_param;

    // Node handle
  ros::NodeHandle m_node_handle;
  ros::Timer m_timer;


  // Published topics
  ros::Publisher m_pub_vehicle_state;
  ros::Publisher m_pub_vehicle_pose;
  ros::Publisher m_pub_vehicle_twist;
  ros::Publisher m_pub_vehicle_odom;

  
  // Subsribed topics
  ros::Subscriber m_sub_input_u;

  // Methods
  bool readConfig();

  void process(const ros::TimerEvent &);

  // Callbacks
  void inputCallback(const mpcc_msgs::Input &input);



public:
    SimpleSim();

    bool init();

    void run();

};


mpcc::State convertState(mpcc_msgs::State state)
{
    mpcc::State x;
    x.X     = state.X;
    x.Y     = state.Y;
    x.phi   = state.phi;
    x.vx    = state.vx;
    x.vy    = state.vy;
    x.r     = state.r;
    x.s     = state.s;
    x.D     = state.D;
    x.delta = state.delta;
    x.vs    = state.vs;

    return x;
}

mpcc_msgs::State convertState(mpcc::State state)
{
    mpcc_msgs::State x;
    x.X     = state.X;
    x.Y     = state.Y;
    x.phi   = state.phi;
    x.vx    = state.vx;
    x.vy    = state.vy;
    x.r     = state.r;
    x.s     = state.s;
    x.D     = state.D;
    x.delta = state.delta;
    x.vs    = state.vs;

    return x;
}


#endif // _SIMPLE_SIM_H_

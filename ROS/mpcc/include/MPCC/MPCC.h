#ifndef _MPCC_H_
#define _MPCC_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>


#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>




#include <cmath>
#include <vector>


#include <stdlib.h>

#include <sstream>
#include <fstream>
#include <string>




#include "../C++/MPC/mpc.h"
#include "../C++/Model/integrator.h"
#include "../C++/Params/track.h"


#include <mpcc_msgs/Input.h>
#include <mpcc_msgs/State.h>
#include <mpcc_msgs/Trajectory.h>

#include <tf/tf.h>


#include <nlohmann/json.hpp>
using json = nlohmann::json;

using namespace mpcc;


class MPCC {
  
  // Attributes
  bool m_system_ready;
  bool m_node_initialized;
  double m_rate;


  State m_x;
  Input m_u;
  MPC m_mpc;


  json m_config;
  mpcc::Param m_model_param;
  mpcc::CostParam m_cost_param;
  mpcc::BoundsParam m_bounds_param;
  mpcc::Track m_track;

    // Node handle
  ros::NodeHandle m_node_handle;
  ros::Timer m_timer;


  // Published topics
  ros::Publisher m_pub_input_u;
  ros::Publisher m_pub_mpc_trajectory;
  ros::Publisher m_pub_mpc_trajectory_nav;
  ros::Publisher m_pub_visualization_marker;

  // Subsribed topics
  ros::Subscriber m_sub_vehicle_state;



  // Methods
  bool readConfig();

  void process(const ros::TimerEvent &);

  void visualizeTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);


  // Callbacks
  void vehicleStateCallback(const mpcc_msgs::State &state);



public:
    MPCC();

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

#endif // _MPCC_H_

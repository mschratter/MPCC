
#include "Simple_Sim/SimpleSim.h"

#include <string>
#include <cmath>
#include <stdlib.h>


using ::ros::NodeHandle;


using namespace std;
using namespace mpcc;

SimpleSim::SimpleSim()
        : m_node_initialized(false),
          m_rate(50.0)
{
}


void SimpleSim::run()
{
  if (!m_node_initialized)
  {
    ROS_FATAL("SimpleSim is not initialized. Shutdown.");
    return;
  }
  ros::spin();
}


// Initialize node
bool SimpleSim::init() {

  // create publishers
  m_pub_vehicle_state = 
        m_node_handle.advertise<mpcc_msgs::State>("/simulator/state", 1);

  m_pub_vehicle_pose = 
        m_node_handle.advertise<geometry_msgs::PoseStamped>("/simulator/pose", 1);

  m_pub_vehicle_twist = 
        m_node_handle.advertise<geometry_msgs::TwistStamped>("/simulator/twist", 1);

  m_pub_vehicle_odom = 
        m_node_handle.advertise<nav_msgs::Odometry>("/simulator/odom", 1);


  // create subscribers
  m_sub_input_u = m_node_handle.subscribe("/simulator/input", 1,
      &SimpleSim::inputCallback,
      this, ros::TransportHints().tcpNoDelay(true));

  m_x.setZero();
 
  if (readConfig()) {
    m_node_initialized = true;
  }

  m_timer = m_node_handle.createTimer(ros::Duration(1.0 / m_rate),
                                        &SimpleSim::process, this);

  m_integrator.setParam(m_model_param);

  mpcc_msgs::State state = convertState(m_x);
  state.header.stamp = ros::Time::now();
  state.header.frame_id = "map";
  m_pub_vehicle_state.publish(state);

  m_system_ready = true;
  return m_node_initialized;
}


bool SimpleSim::readConfig()
{
  ros::NodeHandle priv_nh("~");

  // Config arguments
  priv_nh.param("rate", m_rate, 50.0);
  if (m_rate < 0.1)
  {
    ROS_ERROR_STREAM("Invalid rate < 0.1");
    return false;
  }

  priv_nh.param("x", m_x.X, 0.0);
  priv_nh.param("y", m_x.Y, 0.0);
  priv_nh.param("phi", m_x.phi, 0.0);

  std::string model_file_name;

  if (priv_nh.getParam("model_file_name", model_file_name)) {
    ROS_INFO("Got param from file: %s", model_file_name.c_str());
  }
  else {
    ROS_ERROR("No file for param defined!");
    return false;
  }
  m_model_param = Param("/home/nvidia/mpcc_ws2/src/MPCC/C++/Params/model.json");

  return true;
}


void SimpleSim::process(const ros::TimerEvent &)
{
  m_x = m_integrator.simTimeStep(m_x, m_u, 1.0/m_rate);

  double v_ego = sqrt(m_x.vx*m_x.vx + m_x.vy*m_x.vy);

  mpcc_msgs::State state;
  state = convertState(m_x);
  state.header.stamp = ros::Time::now();
  state.header.frame_id = "map";

  m_pub_vehicle_state.publish(state);

  geometry_msgs::PoseStamped simulator_pose;
  simulator_pose.header = state.header;
  simulator_pose.pose.position.x = state.X;
  simulator_pose.pose.position.y = state.Y;
  simulator_pose.pose.position.z = 0;

  tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, state.phi);
  simulator_pose.pose.orientation.x = quat.x();
  simulator_pose.pose.orientation.y = quat.y();
  simulator_pose.pose.orientation.z = quat.z();
  simulator_pose.pose.orientation.w = quat.w();


  m_pub_vehicle_pose.publish(simulator_pose);

  geometry_msgs::TwistStamped simulator_twist;
  simulator_twist.header.stamp = ros::Time::now();
  simulator_twist.header.frame_id = "base_link";
  simulator_twist.twist.linear.x = v_ego;

  m_pub_vehicle_twist.publish(simulator_twist);

  nav_msgs::Odometry odom;
  odom.header = simulator_pose.header;
  odom.pose.pose = simulator_pose.pose;
  odom.twist.twist = simulator_twist.twist;
  m_pub_vehicle_odom.publish(odom);
}


void SimpleSim::inputCallback(const mpcc_msgs::Input &input_u)
{
  mpcc::Input u;
  m_u.dD = input_u.dD;
  m_u.dDelta = input_u.dDelta;
  m_u.dVs = input_u.dVs;
}


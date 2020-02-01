
#include "MPCC/MPCC.h"

#include <string>
#include <cmath>
#include <stdlib.h>


using ::ros::NodeHandle;


using namespace std;
using namespace mpcc;

MPCC::MPCC()
        : m_node_initialized(false),
          m_rate(50.0)
{
}

void MPCC::run()
{
  if (!m_node_initialized)
  {
    ROS_FATAL("MPCC is not initialized. Shutdown.");
    return;
  }
  ros::spin();
}


// Initialize node
bool MPCC::init() {

  // create publishers
  m_pub_input_u = 
        m_node_handle.advertise<mpcc_msgs::Input>("/simulator/input", 1);

  m_pub_mpc_trajectory = 
          m_node_handle.advertise<mpcc_msgs::Trajectory>("/mpcc/trajectory", 1);

  m_pub_mpc_trajectory_nav = 
          m_node_handle.advertise<nav_msgs::Path>("/mpcc/trajectory_nav", 1);

  m_pub_visualization_marker = 
          m_node_handle.advertise<visualization_msgs::MarkerArray>("/visualization_marker_simple_sim", 10000, true);

  // create subscribers
  m_sub_vehicle_state = m_node_handle.subscribe("/simulator/state", 1,
      &MPCC::vehicleStateCallback,
      this, ros::TransportHints().tcpNoDelay(true));


  m_x.setZero();

  if (readConfig())
  {
    m_node_initialized = true;
  }

  m_timer = m_node_handle.createTimer(ros::Duration(1.0 / m_rate),
                                        &MPCC::process, this);

  TrackPos track_xy = m_track.getTrack();  
  visualizeTrack(track_xy.X, track_xy.Y);

  m_mpc.setConfig(m_config["n_sqp"], m_config["n_reset"], m_config["sqp_mixing"], m_model_param, m_cost_param, m_bounds_param);
  m_mpc.setTrack(track_xy.X, track_xy.Y);


  m_system_ready = true;
  return m_node_initialized;
}



bool MPCC::readConfig()
{
  ros::NodeHandle priv_nh("~");

  // Config arguments
  priv_nh.param("rate", m_rate, 50.0);
  if (m_rate < 0.1) {
      ROS_ERROR_STREAM("Invalid rate < 0.1");
      return false;
  }

  std::string model_file_name;
  if (priv_nh.getParam("model_file_name", model_file_name)) {
    ROS_INFO("Got param from file: %s", model_file_name.c_str());
  }
  else {
    ROS_ERROR("No file for param defined!");
    return false;
  }
  m_model_param = Param(model_file_name);

  std::string cost_file_name;
  if (priv_nh.getParam("cost_file_name", cost_file_name)) {
    ROS_INFO("Got costs from file: %s", cost_file_name.c_str());
  }
  else {
    ROS_ERROR("No file for costs defined!");
    return false;
  }
  m_cost_param = CostParam(cost_file_name);


  std::string bounds_file_name;
  if (priv_nh.getParam("bounds_file_name", bounds_file_name)) {
    ROS_INFO("Got bounds from file: %s", bounds_file_name.c_str());
  }
  else {
    ROS_ERROR("No file for bounds defined!");
    return false;
  }
  m_bounds_param = BoundsParam(bounds_file_name);


  std::string track_file_name;
  if (priv_nh.getParam("track_file_name", track_file_name)) {
    ROS_INFO("Got track from file: %s", track_file_name.c_str());
  }
  else {
    ROS_ERROR("No file for track defined!");
    return false;
  }
  m_track = Track(track_file_name);

  std::ifstream iConfig("/home/nvidia/mpcc_ws2/src/MPCC/C++/Params/config.json");
  iConfig >> m_config;

  return true;
}


void MPCC::process(const ros::TimerEvent &)
{

}



void MPCC::vehicleStateCallback(const mpcc_msgs::State &state_simulator)
{
  static int alive = 0;

  m_x = convertState(state_simulator);
  MPCReturn mpc_sol = m_mpc.runMPC(m_x);
  cout << "Time horizon: " << mpc_sol.time_total << endl;;

  mpcc_msgs:: Trajectory mpc_trajectory;
  mpc_trajectory.header.frame_id = "map";
  mpc_trajectory.header.stamp = ros::Time::now();

  nav_msgs::Path mpc_path;
  mpc_path.header = mpc_trajectory.header;
  
  auto x = mpc_sol.mpc_horizon[0];
  geometry_msgs::PoseStamped pose;
  pose.header = mpc_path.header;
  for (int i = 0; i < mpc_sol.mpc_horizon.size(); i++ ) {
    State X = mpc_sol.mpc_horizon[i].xk;
    pose.pose.position.x = X.X;
    pose.pose.position.y = X.Y;
    pose.pose.position.z = 0;
    pose.pose.orientation.w = 1;

    tf::Quaternion quat = tf::createQuaternionFromRPY(0, 0, X.phi);
    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();
    mpc_path.poses.push_back(pose);


    mpc_trajectory.x.push_back(X.X);
    mpc_trajectory.y.push_back(X.Y);
    mpc_trajectory.phi.push_back(X.phi);
    mpc_trajectory.k.push_back(0);
    mpc_trajectory.v.push_back(0);
    mpc_trajectory.vx.push_back(X.vx);
    mpc_trajectory.vy.push_back(X.vy);
    mpc_trajectory.a.push_back(0);

  }
  mpc_trajectory.counter = alive;
  //mpc_trajectory.status = mpcc_msgs::Trajectory::....
  mpc_trajectory.prediction_length = 100;

  m_pub_mpc_trajectory.publish(mpc_trajectory);
  m_pub_mpc_trajectory_nav.publish(mpc_path);

  mpcc_msgs::Input input_u;
  input_u.header.frame_id = "mpcc";
  input_u.header.stamp = ros::Time::now();
  input_u.dD = mpc_sol.u0.dD;
  input_u.dVs = mpc_sol.u0.dVs;
  input_u.dDelta = mpc_sol.u0.dDelta;
  m_pub_input_u.publish(input_u);

  alive++;
}

void MPCC::visualizeTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker_visualization;
  marker_visualization.ns = "simple_sim";

  marker_visualization.header.frame_id = "map";
  marker_visualization.header.stamp = ros::Time::now();
  marker_visualization.lifetime = ros::Duration();
  marker_visualization.type = visualization_msgs::Marker::LINE_STRIP;
  marker_visualization.action = visualization_msgs::Marker::ADD;

  marker_visualization.color.a = 0.5;
  marker_visualization.color.r = 1.0; marker_visualization.color.g = 1.0; marker_visualization.color.b = 1.0;

  marker_visualization.scale.x = 0.02;
  marker_visualization.pose.orientation.w = 1;
 
  geometry_msgs::Point last_point, current_point;
  last_point.z = 0;
  current_point.z = 0;

  marker_visualization.id =  1;
  marker_visualization.points.clear();
  marker_visualization.colors.clear();

  for (int i = 1; i < X.size() && i < Y.size(); i = i + 2 ) {
    last_point.x = X[i-1]; 
    last_point.y = Y[i-1];
    marker_visualization.points.push_back(last_point);

    current_point.x = X[i];
    current_point.y = Y[i];
    marker_visualization.points.push_back(current_point);
  }
  marker_array.markers.push_back(marker_visualization);
  m_pub_visualization_marker.publish(marker_array);
}


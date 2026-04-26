#include "uav_city_navigation/geometric_controller.h"
#include "uav_city_navigation/jerk_tracking_control.h"
#include "uav_city_navigation/nonlinear_attitude_control.h"
#include "uav_city_navigation/nonlinear_geometric_control.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private), node_state(WAITING_FOR_HOME_POSE)
{
  referenceSub_ =
      nh_.subscribe("reference/setpoint", 1, &geometricCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ =
      nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavstateSub_ =
      nh_.subscribe("mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());
  ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback,
                                   this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback,
                                      this); // Define timer for constant loop rate

  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);

  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<int>("ctrl_mode_mine", ctrl_mode_mine_, GEO_CONTROLLER_WITHOUT_POSITION_FB);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);

  double dx, dy, dz;
  nh_private_.param<double>("drag_dx", dx, 0.0);
  nh_private_.param<double>("drag_dy", dy, 0.0);
  nh_private_.param<double>("drag_dz", dz, 0.0);
  D_ << dx, dy, dz;

  double attctrl_tau;
  nh_private_.param<double>("attctrl_constant", attctrl_tau, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05); // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);   // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);

  // --- Setup Multi-Agent Topology via Adjacency Matrix ---
  nh_private_.param<int>("uav_id", uav_id_, 0);
  nh_private_.param<int>("num_uavs", num_uavs_, 3);
  // Fetch the spawn offsets for ALL drones to build a global map
  for(int i = 0; i < num_uavs_; i++) {
      double ox = 0.0, oy = 0.0, oz = 0.0;
      nh_.param("/uav" + std::to_string(i) + "/geometric_controller/spawn_x", ox, 0.0);
      nh_.param("/uav" + std::to_string(i) + "/geometric_controller/spawn_y", oy, 0.0);
      nh_.param("/uav" + std::to_string(i) + "/geometric_controller/spawn_z", oz, 0.0);
      spawn_offsets_[i] = Eigen::Vector3d(ox, oy, oz);
  }

  std::vector<int> adj_matrix;
  // Fetch the global adjacency matrix (note the leading "/" indicating global namespace)
  if (nh_.getParam("/adjacency_matrix", adj_matrix))
  {
    if (adj_matrix.size() != num_uavs_ * num_uavs_)
    {
      ROS_ERROR("[UAV %d] Topology error: Adjacency matrix size (%lu) does not match num_uavs^2 (%d)", 
                uav_id_, adj_matrix.size(), num_uavs_ * num_uavs_);
    }
    else
    {
      // Read row 'uav_id_' to find which neighbors to subscribe to
      for (int j = 0; j < num_uavs_; ++j)
      {
        int a_ij = adj_matrix[uav_id_ * num_uavs_ + j]; 
        
        if (a_ij == 1 && j != uav_id_) // If there is a directed edge from j to i
        {
          std::string pose_topic = "/uav" + std::to_string(j) + "/mavros/local_position/pose";
          std::string twist_topic = "/uav" + std::to_string(j) + "/mavros/local_position/velocity_local";

          neighbor_pose_subs_[j] = nh_.subscribe<geometry_msgs::PoseStamped>(
              pose_topic, 1, boost::bind(&geometricCtrl::neighborPoseCallback, this, _1, j));
              
          neighbor_twist_subs_[j] = nh_.subscribe<geometry_msgs::TwistStamped>(
              twist_topic, 1, boost::bind(&geometricCtrl::neighborTwistCallback, this, _1, j));
              
          ROS_INFO("[UAV %d] Topology Edge Added: Subscribed to UAV %d", uav_id_, j);
        }
      }
    }
  }
  else
  {
    ROS_ERROR("[UAV %d] Could not find /adjacency_matrix on the parameter server!", uav_id_);
  }

  nh_private_.param<double>("Omegad_x_", Omega_d_x_, 0.30);
  nh_private_.param<double>("Omegad_y_", Omega_d_y_, 0.0);
  nh_private_.param<double>("Omegad_z_", Omega_d_z_, 0.75);
  nh_private_.param<double>("Kperp_", K_perp_, 1.75);
  nh_private_.param<double>("Kpapral_", K_papral_, 0.4);
  nh_private_.param<double>("radius", radius_, 5.0);

  nh_private_.param<double>("k2", k2_, 0.3);
  nh_private_.param<double>("k3", k3_, 0.5);
  nh_private_.param<double>("k4", k4_, 0.6);

  spin_axis_pub_ = nh_.advertise<visualization_msgs::Marker>("geometric_controller/spin_axis", 1);
  global_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("geometric_controller/global_pose", 1);

  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 2.0);

  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_; // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  Omega_d_ << Omega_d_x_, Omega_d_y_, Omega_d_z_;

  bool jerk_enabled = false;
  if (!jerk_enabled)
  {
    if (ctrl_mode_ == ERROR_GEOMETRIC)
    {
      controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
    }
    else
    {
      controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
    }
  }
  else
  {
    controller_ = std::make_shared<JerkTrackingControl>();
  }

  global_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("geometric_controller/global_odom", 1);

  double spawn_x, spawn_y, spawn_z;
  nh_private_.param<double>("spawn_x", spawn_x, 4.0);
  nh_private_.param<double>("spawn_y", spawn_y, -40.0);
  nh_private_.param<double>("spawn_z", spawn_z, 200.0);
  spawn_offsets_[uav_id_] = Eigen::Vector3d(spawn_x, spawn_y, spawn_z);
  
  trajectory_received_ = false;
}
geometricCtrl::~geometricCtrl()
{
  // Destructor
}

void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped &msg)
{
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg)
{
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);

  if (msg.type_mask == 1)
  {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = Eigen::Vector3d::Zero();
  }
  else if (msg.type_mask == 2)
  {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
  }
  else if (msg.type_mask == 4)
  {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
  }
  else
  {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = toEigen(msg.snap);
  }
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg)
{
  if (!velocity_yaw_)
    mavYaw_ = double(msg.data);
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg)
{
  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];

  trajectory_received_ = true; 
  
  // Directly accept the trajectory position
  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  
  // REMOVE OR COMMENT OUT THIS LINE:
  // targetPos_ -= spawn_offsets_[uav_id_];

  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();

  if (!velocity_yaw_)
  {
    Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
                         pt.transforms[0].rotation.z);
    Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2); // RPY
    mavYaw_ = rpy(2);
  }
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!received_home_pose)
  {
    received_home_pose = true;
    home_pose_ = msg.pose;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  
  // MUST remain strictly local for the flight controller to stay stable!
  mavPos_(0) = msg.pose.position.x;
  mavPos_(1) = msg.pose.position.y;
  mavPos_(2) = msg.pose.position.z;
  
  mavAtt_(0) = msg.pose.orientation.w;
  mavAtt_(1) = msg.pose.orientation.x;
  mavAtt_(2) = msg.pose.orientation.y;
  mavAtt_(3) = msg.pose.orientation.z;

  // --- NEW: Publish Global Pose for RViz ---
  Eigen::Vector3d global_pos = mavPos_ + spawn_offsets_[uav_id_];
  geometry_msgs::PoseStamped global_msg = vector3d2PoseStampedMsg(global_pos, mavAtt_);
  global_pose_pub_.publish(global_msg);

  nav_msgs::Odometry global_odom;
  global_odom.header.stamp = ros::Time::now();
  global_odom.header.frame_id = "map";
  global_odom.pose.pose.position.x = global_pos(0);
  global_odom.pose.pose.position.y = global_pos(1);
  global_odom.pose.pose.position.z = global_pos(2);
  global_odom.pose.pose.orientation = msg.pose.orientation;
  global_odom_pub_.publish(global_odom);
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg)
{
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
}

bool geometricCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
  node_state = LANDING;
  return true;
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event)
{
  switch (node_state)
  {
  case WAITING_FOR_HOME_POSE:
    waitForPredicate(&received_home_pose, "Waiting for home pose...");
    ROS_INFO("Got pose! Drone Ready to be armed.");

    // Hardcode a safe 2.0m vertical takeoff
    takeoff_pos_ << 0.0, 0.0, 2.0;

    node_state = TAKEOFF;
    break;

  case TAKEOFF:
  {
    Eigen::Vector3d pos_error = mavPos_ - takeoff_pos_;
    Eigen::Vector3d a_des = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * mavVel_ - gravity_;
    computeBodyRateCmd(cmdBodyRate_, a_des);
    pubRateCommands(cmdBodyRate_, q_des);

    // If altitude reached and velocity is low, switch to HOVERING
    if (std::abs(pos_error(2)) < 0.5 && mavVel_.norm() < 0.5)
    {
      ROS_INFO_ONCE("Takeoff altitude reached. Hovering and waiting for trajectory...");
      node_state = HOVERING;
    }
    break;
  }

  case HOVERING:
  {
    // Maintain hover position
    Eigen::Vector3d pos_error = mavPos_ - takeoff_pos_;
    Eigen::Vector3d a_des = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * mavVel_ - gravity_;
    computeBodyRateCmd(cmdBodyRate_, a_des);
    pubRateCommands(cmdBodyRate_, q_des);

    // Wait for the user to draw the path in RViz
    if (trajectory_received_)
    {
      ROS_INFO("Trajectory received! Executing mission.");
      node_state = MISSION_EXECUTION;
    }
    break;
  }

  case MISSION_EXECUTION:
  {
    ROS_INFO_ONCE("Executing mission...");
    Eigen::Vector3d desired_acc;
    if (feedthrough_enable_)
    {
      desired_acc = targetAcc_;
    }
    else
    {
      desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
    }
    computeBodyRateCmd(cmdBodyRate_, desired_acc);
    pubReferencePose(targetPos_, q_des);
    pubRateCommands(cmdBodyRate_, q_des);
    appendPoseHistory();
    pubPoseHistory();
    break;
  }

  case LANDING:
  {
    geometry_msgs::PoseStamped landingmsg;
    landingmsg.header.stamp = ros::Time::now();
    landingmsg.pose = home_pose_;
    landingmsg.pose.position.z = landingmsg.pose.position.z + 1.0;
    target_pose_pub_.publish(landingmsg);
    node_state = LANDED;
    ros::spinOnce();
    break;
  }
  case LANDED:
    ROS_INFO("Landed. Please set to position control and disarm.");
    cmdloop_timer_.stop();
    break;
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event)
{
  if (sim_enable_)
  {
    // Enable OFFBoard mode and arm automatically
    // This will only run if the vehicle is simulated
    mavros_msgs::SetMode offb_set_mode;
    arm_cmd_.request.value = true;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
    {
      if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent)
      {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    }
    else
    {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0)))
      {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success)
        {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude)
{
  geometry_msgs::PoseStamped msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_.publish(msg);
}

void geometricCtrl::pubSpinAxis()
{
  double Omega_d_norm_sq = Omega_d_.dot(Omega_d_);
  if (Omega_d_norm_sq < 1e-8) return; // Prevent division by zero if Omega is 0

  // 1. Calculate the rotation center 'c' in the global frame
  // c = p + (omega x v) / |omega|^2
  // Inside pubSpinAxis()...
  // Shift the spin axis center to global coordinates
  Eigen::Vector3d global_pos = mavPos_ + spawn_offsets_[uav_id_];
  Eigen::Vector3d c = global_pos + (Omega_d_.cross(mavVel_) / Omega_d_norm_sq);

  // 2. Setup the RViz Line Marker
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = "map";
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = "spin_axis";
  line_marker.id = uav_id_; 
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.scale.x = 0.05; // Line thickness

  // 3. Set Color matching the UAV's path color
  line_marker.color.a = 0.8; 
  if (uav_id_ == 0)      { line_marker.color.r = 1.0; line_marker.color.g = 0.0;  line_marker.color.b = 0.0; } // Red
  else if (uav_id_ == 1) { line_marker.color.r = 0.0; line_marker.color.g = 0.66; line_marker.color.b = 1.0; } // Blue/Cyan
  else if (uav_id_ == 2) { line_marker.color.r = 1.0; line_marker.color.g = 1.0;  line_marker.color.b = 0.0; } // Yellow

  // 4. Stretch the line through 'c' parallel to Omega_d_
  // Extending 20 meters in both directions to make it "as long as possible" visually
  Eigen::Vector3d dir = Omega_d_.normalized();
  Eigen::Vector3d p1 = c - (dir * 20.0);
  Eigen::Vector3d p2 = c + (dir * 20.0);

  geometry_msgs::Point pt1, pt2;
  pt1.x = p1(0); pt1.y = p1(1); pt1.z = p1(2);
  pt2.x = p2(0); pt2.y = p2(1); pt2.z = p2(2);

  line_marker.points.push_back(pt1);
  line_marker.points.push_back(pt2);

  spin_axis_pub_.publish(line_marker);
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude)
{
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128; // Ignore orientation messages
  msg.orientation.w = target_attitude(0);
  msg.orientation.x = target_attitude(1);
  msg.orientation.y = target_attitude(2);
  msg.orientation.z = target_attitude(3);
  msg.thrust = cmd(3);

  angularVelPub_.publish(msg);
}

void geometricCtrl::pubPoseHistory()
{
  nav_msgs::Path msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;

  posehistoryPub_.publish(msg);
}

void geometricCtrl::pubSystemStatus()
{
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196; // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

void geometricCtrl::appendPoseHistory()
{
  // Shift the colored RViz trail to global coordinates
  Eigen::Vector3d global_pos = mavPos_ + spawn_offsets_[uav_id_];
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(global_pos, mavAtt_));
  if (posehistory_vector_.size() > posehistory_window_) {
    posehistory_vector_.pop_back();
  }
}

geometry_msgs::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                                                  Eigen::Vector4d &orientation)
{
  geometry_msgs::PoseStamped encode_msg;
  encode_msg.header.stamp = ros::Time::now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc)
{
  Eigen::Vector3d a_des = Eigen::Vector3d::Zero();

  switch (ctrl_mode_mine_)
  {
  case ORIGINAL_GEO_CONTROLLER:
  {
    // Compute BodyRate commands using differential flatness
    // Controller based on Faessler 2017
    const Eigen::Vector3d a_ref = target_acc;
    if (velocity_yaw_)
    {
      // SAFETY CONDITION: Only update heading if the trajectory demands movement.
      // Otherwise, hold the last known heading to prevent violent spinning.
      if (target_vel.norm() > 0.1) {
          mavYaw_ = getVelocityYaw(target_vel); // Use clean target velocity
      } else if (mavVel_.norm() > 0.2) {
          mavYaw_ = getVelocityYaw(mavVel_);    // Fallback to actual velocity
      }
    }
    const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, mavYaw_);
    
    const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);
    const Eigen::Vector3d pos_error = mavPos_ - target_pos;
    const Eigen::Vector3d vel_error = mavVel_ - target_vel;
    // Position Controller
    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);
    // Rotor Drag compensation
    const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel; // Rotor drag
    // Reference acceleration
    a_des = a_fb + a_ref - a_rd - gravity_;
    break;
  }

  case GEO_CONTROLLER_WITHOUT_POSITION_FB:
  {
    double Omega_d_norm = Omega_d_.dot(Omega_d_); 
    Eigen::Vector3d v_perp;
    if (Omega_d_norm > 1e-8)
    { // Safety check to prevent division by zero
      v_perp = (mavVel_.dot(Omega_d_) / Omega_d_norm) * Omega_d_;
    }
    else
    {
      v_perp = Eigen::Vector3d::Zero();
    }
    Eigen::Vector3d v_paral = mavVel_ - v_perp;
    if (v_paral.norm() < 1e-8)
    {
      v_paral = Omega_d_.cross(0.1 * Eigen::Vector3d::Ones());
    }
    Eigen::Vector3d e_paralv = (radius_ * radius_ * Omega_d_norm - v_paral.dot(v_paral)) * v_paral;
    a_des = Omega_d_.cross(v_paral) - K_perp_ * v_perp + K_papral_ * e_paralv - gravity_;
    // Print values to the terminal twice a second (every 0.5s)
    ROS_INFO_STREAM_THROTTLE(0.5,
                             "\n--- Controller Tuning ---"
                                 << "\nOmega_d: [" << Omega_d_(0) << ", " << Omega_d_(1) << ", " << Omega_d_(2) << "]"
                                 << "\nv_perp:  [" << v_perp(0) << ", " << v_perp(1) << ", " << v_perp(2) << "]"
                                 << "\nv_paral: [" << v_paral(0) << ", " << v_paral(1) << ", " << v_paral(2) << "]"
                                 << "\na_des:   [" << a_des(0) << ", " << a_des(1) << ", " << a_des(2) << "]");
    break;
  }

  case GEO_CONTROLLER_WITH_POSITION_FB:
  {
    // --- 0. Time Tracking for Moving Target ---
    // Record the time this mode was first activated to act as t = 0
    static bool is_first_run = true;
    static double mode_start_time = 0.0;
    if (is_first_run) {
        mode_start_time = ros::Time::now().toSec();
        is_first_run = false;
    }
    double t = ros::Time::now().toSec() - mode_start_time;

    // --- 1. Target Tracking Definitions ---
    // Base position
    Eigen::Vector3d rc(0.0, -20.0, 20.0);
    Eigen::Vector3d rc_dot(0.0, 0.0, 0.0);
    Eigen::Vector3d rc_ddot(0.0, 0.0, 0.0);

    // Stays at (0,0,20) for 15 seconds, then starts moving
    double t_move = 15.0; 
    if (t > t_move) {
        double dt = t - t_move;
        
        // Add the position offset: 0.1*dt, 0.2*dt, 0.05*dt
        rc(0) += 0.3 * dt;
        rc(1) += 0.6 * dt;
        rc(2) += 0.15 * dt;

        // The derivative (velocity) of the above offset
        rc_dot << 0.3, 0.6, 0.15;
        
        // rc_ddot remains (0, 0, 0) because the velocity is constant
    }

    // Calculate Relative States
    Eigen::Vector3d xr = mavPos_ - rc;
    Eigen::Vector3d vr = mavVel_ - rc_dot;

    // --- 2. Projection Matrices ---
    double Omega_d_norm_sq = Omega_d_.dot(Omega_d_);
    Eigen::Matrix3d QQ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d PP = Eigen::Matrix3d::Identity();

    Eigen::Vector3d vr_perp = Eigen::Vector3d::Zero();
    Eigen::Vector3d xr_perp = Eigen::Vector3d::Zero();

    if (Omega_d_norm_sq > 1e-8) {
      QQ = (Omega_d_ * Omega_d_.transpose()) / Omega_d_norm_sq;
      PP = Eigen::Matrix3d::Identity() - QQ;
      vr_perp = QQ * vr;
      xr_perp = QQ * xr;
    }

    Eigen::Vector3d vr_paral = PP * vr;
    if (vr_paral.norm() < 1e-8) {
      vr_paral = Omega_d_.cross(0.1 * Eigen::Vector3d::Ones());
    }

    // --- 3. Position Feedback (Moving Target) ---
    double kq = k3_;  // Gain to pull the orbit center to rc (in-plane)
    double kpx = k2_; // Gain to pull the UAV into the correct orthogonal plane (out-of-plane)

    Eigen::Vector3d qr = Eigen::Vector3d::Zero();
    if (Omega_d_norm_sq > 1e-8) {
       qr = xr + Omega_d_.cross(vr) / Omega_d_norm_sq;
    } else {
       qr = xr;
    }

    Eigen::Vector3d pos_corr = kq * Omega_d_.cross(qr) - kpx * xr_perp;

    // --- 4. Desired Acceleration (A) ---
    Eigen::Vector3d e_paralv = (radius_ * radius_ * Omega_d_norm_sq - vr_paral.dot(vr_paral)) * vr_paral;
    Eigen::Vector3d ard = Omega_d_.cross(vr_paral) - K_perp_ * vr_perp + K_papral_ * e_paralv + pos_corr;

    // Absolute desired acceleration
    a_des = ard + rc_ddot - gravity_;

    // --- 5. Terminal Output for Debugging ---
    static ros::Time last_print_time = ros::Time::now();
    if ((ros::Time::now() - last_print_time).toSec() > 0.5) 
    {
      std::stringstream ss;
      ss << "\n--- MOVING TARGET GEO_CONTROLLER ---"
         << "\nTime t:         " << t << " s"
         << "\nTarget pos r_c: [" << rc(0) << ", " << rc(1) << ", " << rc(2) << "]"
         << "\nTarget vel:     [" << rc_dot(0) << ", " << rc_dot(1) << ", " << rc_dot(2) << "]"
         << "\nRel Pos Error:  [" << xr(0) << ", " << xr(1) << ", " << xr(2) << "]"
         << "\nAbs Accel:      [" << a_des(0) << ", " << a_des(1) << ", " << a_des(2) << "]";
      ROS_INFO_STREAM(ss.str());
      last_print_time = ros::Time::now();
    }

    // Update the visual spin axis in RViz
    // (We need to temporarily override targetPos_ so pubSpinAxis knows where the center is)
    targetPos_ = rc; 
    pubSpinAxis();

    break;
  }

  case MULTI_AGENT_GEO_CONTROLLER:
  {
    static bool is_first_run = true;
    static double mode_start_time = 0.0;
    if (is_first_run) {
        mode_start_time = ros::Time::now().toSec();
        is_first_run = false;
    }
    double t = ros::Time::now().toSec() - mode_start_time;

    // --- 1. Target Tracking Definitions (Relative to Local Takeoff) ---
    // We use mavPos_ (local) to avoid 200m coordinate offsets from Gazebo
    Eigen::Vector3d rc(10.0, -10.0, 10.0); // Orbit 20m above takeoff point
    Eigen::Vector3d rc_dot(0.0, 0.0, 0.0);
    Eigen::Vector3d rc_ddot(0.0, 0.0, 0.0);

    double t_move = 15.0; 
    if (t > t_move) {
        double dt = t - t_move;
        rc(0) += 1.0 * dt;
        rc(1) += 1.0 * dt;
        rc(2) += 0.0 * dt;
        rc_dot << 1.0, 1.0, 0.0;
    }

    // States relative to target center (Local Frame)
    Eigen::Vector3d xr = mavPos_ - rc;
    Eigen::Vector3d vr = mavVel_ - rc_dot;

    // --- 2. Projections (Fixed for Eigen Type-Safety) ---
    double Omega_d_norm_sq = Omega_d_.dot(Omega_d_);
    Eigen::Matrix3d QQ = Eigen::Matrix3d::Zero();
    
    if (Omega_d_norm_sq > 1e-8) {
        // Explicitly calculating QQ only if Omega is non-zero
        QQ = (Omega_d_ * Omega_d_.transpose()) / Omega_d_norm_sq;
    }
    Eigen::Matrix3d PP = Eigen::Matrix3d::Identity() - QQ;

    Eigen::Vector3d vr_perp = QQ * vr;
    Eigen::Vector3d vr_paral = PP * vr;
    Eigen::Vector3d xr_perp = QQ * xr;

    if (vr_paral.norm() < 1e-8) {
        vr_paral = Omega_d_.cross(0.1 * Eigen::Vector3d::Ones());
    }

    // --- 3. Position Feedback (Fixed for Eigen Type-Safety) ---
    double kq = Kpos_x_;  
    double kpx = Kpos_z_; 
    
    // Calculate qr safely
    Eigen::Vector3d qr = xr;
    if (Omega_d_norm_sq > 1e-8) {
        qr += (Eigen::Vector3d)(Omega_d_.cross(vr) / Omega_d_norm_sq);
    }

    // CLAMP qr and xr_perp to prevent extreme commands from large distances
    double max_pos_err = 5.0; 
    if (qr.norm() > max_pos_err) qr = qr.normalized() * max_pos_err;
    if (xr_perp.norm() > max_pos_err) xr_perp = xr_perp.normalized() * max_pos_err;

    Eigen::Vector3d pos_corr = kq * Omega_d_.cross(qr) - kpx * xr_perp;

    // --- 4. Base Vector Field (Orbital Velocity) ---
    Eigen::Vector3d e_paralv = (radius_ * radius_ * Omega_d_norm_sq - vr_paral.dot(vr_paral)) * vr_paral;
    Eigen::Vector3d ard_base = Omega_d_.cross(vr_paral) - K_perp_ * vr_perp + K_papral_ * e_paralv + pos_corr;

    // --- 5. Consensus Coupling (Swarm Coordination) ---
    Eigen::Vector3d a_coupling = Eigen::Vector3d::Zero();
    Eigen::Vector3d my_global_pos = mavPos_ + spawn_offsets_[uav_id_];

    for (auto const& pair : neighbor_pos_) {
      int j = pair.first;
      if (neighbor_vel_.find(j) == neighbor_vel_.end()) continue; 
      
      Eigen::Vector3d neighbor_global_pos = pair.second + spawn_offsets_[j];
      Eigen::Vector3d dx_diff = my_global_pos - neighbor_global_pos;
      Eigen::Vector3d dv_diff = mavVel_ - neighbor_vel_[j];

      // Limit consensus influence to avoid "jitter" at large distances
      if (dx_diff.norm() > 10.0) dx_diff = dx_diff.normalized() * 10.0;

      Eigen::Vector3d term1 = k3_ * (PP * dx_diff).cross(Omega_d_); 
      Eigen::Vector3d term2 = k2_ * (QQ * dx_diff);                 
      Eigen::Vector3d term3 = k4_ * dv_diff;                        
      a_coupling += (term1 + term2 + term3);
    }

    // --- 6. Final Command & Safety ---
    Eigen::Vector3d a_cmd = ard_base + rc_ddot - a_coupling;
    if (a_cmd.norm() > max_fb_acc_) a_cmd = a_cmd.normalized() * max_fb_acc_;

    a_des = a_cmd - gravity_;

    // ALTITUDE PRIORITY: Never let the drone fall due to aggressive horizontal demand
    // 9.0 ensures it is always attempting to fight gravity
    if (a_des(2) < 9.0) a_des(2) = 9.0; 

    targetPos_ = rc;
    pubSpinAxis();
    break;
  }

  default:
  {
    ROS_WARN("Invalid control mode! Defaulting to original geometric controller.");
    break;
  }
  }

  return a_des;
}

void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des)
{
  // Reference attitude
  q_des = acc2quaternion(a_des, mavYaw_);

  controller_->Update(mavAtt_, q_des, a_des, targetJerk_); // Calculate BodyRate
  bodyrate_cmd.head(3) = controller_->getDesiredRate();
  double thrust_command = controller_->getDesiredThrust().z();
  bodyrate_cmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command +
                                      norm_thrust_offset_)); // Calculate thrustcontroller_->getDesiredThrust()(3);
}

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error)
{
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error; // feedforward term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb; // Clip acceleration if reference is too large

  return a_fb;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw)
{
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
  return true;
}

void geometricCtrl::dynamicReconfigureCallback(uav_city_navigation::GeometricControllerConfig &config,
                                               uint32_t level)
{
  if (max_fb_acc_ != config.max_acc)
  {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  }
  // --- YOUR NEW CUSTOM PARAMETERS ---
  else if (Omega_d_x_ != config.Omegad_x)
  {
    Omega_d_x_ = config.Omegad_x;
    ROS_INFO("Reconfigure request : Omegad_x  = %.2f  ", config.Omegad_x);
  }
  else if (Omega_d_y_ != config.Omegad_y)
  {
    Omega_d_y_ = config.Omegad_y;
    ROS_INFO("Reconfigure request : Omegad_y  = %.2f  ", config.Omegad_y);
  }
  else if (Omega_d_z_ != config.Omegad_z)
  {
    Omega_d_z_ = config.Omegad_z;
    ROS_INFO("Reconfigure request : Omegad_z  = %.2f  ", config.Omegad_z);
  }
  else if (K_perp_ != config.Kperp)
  {
    K_perp_ = config.Kperp;
    ROS_INFO("Reconfigure request : Kperp  = %.2f  ", config.Kperp);
  }
  else if (K_papral_ != config.Kpapral)
  {
    K_papral_ = config.Kpapral;
    ROS_INFO("Reconfigure request : Kpapral  = %.2f  ", config.Kpapral);
  }
  else if (radius_ != config.radius)
  {
    radius_ = config.radius;
    ROS_INFO("Reconfigure request : radius  = %.2f  ", config.radius);
  }
  else if (k2_ != config.k2) {
    k2_ = config.k2;
    ROS_INFO("Reconfigure request : k2 = %.2f", config.k2);
  }
  else if (k3_ != config.k3) {
    k3_ = config.k3;
    ROS_INFO("Reconfigure request : k3 = %.2f", config.k3);
  }
  else if (k4_ != config.k4) {
    k4_ = config.k4;
    ROS_INFO("Reconfigure request : k4 = %.2f", config.k4);
  }

  // Rebuild the Eigen Vectors using the newly updated scalars
  Omega_d_ << Omega_d_x_, Omega_d_y_, Omega_d_z_;
}

void geometricCtrl::neighborPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int neighbor_id)
{
  neighbor_pos_[neighbor_id] = toEigen(msg->pose.position);
}

void geometricCtrl::neighborTwistCallback(const geometry_msgs::TwistStamped::ConstPtr &msg, int neighbor_id)
{
  neighbor_vel_[neighbor_id] = toEigen(msg->twist.linear);
}

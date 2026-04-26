#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

// Useful customized headers
#include "uav_city_navigation/trajectory_generator_waypoint.h"

using namespace std;
using namespace Eigen;

// Param from launch file
double _vis_traj_width;
double _Vel, _Acc;
int _dev_order, _min_order;

// ros related
ros::Subscriber _odom_sub;
ros::Subscriber _way_pts_sub;
ros::Publisher _wp_traj_vis_pub, _wp_path_vis_pub;

ros::Publisher _cmd_traj_pub;
ros::Timer _traj_sampler_timer;
bool _trajectory_active = false;
ros::Time _traj_start_time;

// for planning
int _poly_num1D;
MatrixXd _polyCoeff;
VectorXd _polyTime;
Vector3d _startPos  = Vector3d::Zero();
Vector3d _startVel  = Vector3d::Zero();
Vector3d _startAcc  = Vector3d::Zero();
Vector3d _endVel    = Vector3d::Zero();
Vector3d _endAcc    = Vector3d::Zero();


// declare
void visWayPointTraj(MatrixXd polyCoeff, VectorXd time);
void visWayPointPath(MatrixXd path);
Vector3d getPosPoly(MatrixXd polyCoeff, int k, double t);
VectorXd timeAllocation(MatrixXd Path);
void trajGeneration(Eigen::MatrixXd path);
void rcvWaypointsCallBack(const nav_msgs::Path &wp);

Vector3d getVelPoly(MatrixXd polyCoeff, int k, double t);
Vector3d getAccPoly(MatrixXd polyCoeff, int k, double t);
void trajSamplerCallback(const ros::TimerEvent& event);

//Get the path points 
void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        // FIX: Override RViz Z=0 clicks with the drone's actual hover altitude
        Vector3d pt( wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, _startPos(2));
        
        // Safety filter to prevent division-by-zero crashes
        if ((pt - _startPos).norm() < 0.2) {
            continue;
        }
        
        wp_list.push_back(pt);
        ROS_INFO("Valid Flat Waypoint %d: (%f, %f, %f)", k+1, pt(0), pt(1), pt(2));
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);
    waypoints.row(0) = _startPos;

    for(int k = 0; k < (int)wp_list.size(); k++)
        waypoints.row(k+1) = wp_list[k];

    trajGeneration(waypoints);
}

void trajGeneration(Eigen::MatrixXd path)
{
    ros::Time time_start = ros::Time::now();

    TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;
    
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;

    // use "trapezoidal velocity" time allocation
    _polyTime  = timeAllocation(path);

    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

    ros::Time time_end = ros::Time::now();
    ROS_WARN("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);

    visWayPointPath(path);    // visulize path
    visWayPointTraj( _polyCoeff, _polyTime);    // visulize trajectory

    _traj_start_time = ros::Time::now();
    _trajectory_active = true;
    ROS_INFO("Trajectory Active! Publishing to Geometric Controller at 100Hz");
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Continuously update the starting constraints to match the drone's true state
    _startPos(0) = msg->pose.pose.position.x;
    _startPos(1) = msg->pose.pose.position.y;
    _startPos(2) = msg->pose.pose.position.z;
    
    _startVel(0) = msg->twist.twist.linear.x;
    _startVel(1) = msg->twist.twist.linear.y;
    _startVel(2) = msg->twist.twist.linear.z;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel", _Vel, 1.0);
    nh.param("planning/acc", _Acc, 1.0);
    nh.param("planning/dev_order", _dev_order, 3);  // the order of derivative, _dev_order = 3->minimum jerk, _dev_order = 4->minimum snap
    nh.param("planning/min_order", _min_order, 3);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    //_poly_numID is the maximum order of polynomial
    _poly_num1D = 2 * _dev_order;

    _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );
    _odom_sub        = nh.subscribe( "odom", 1, odomCallback );

    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

    _cmd_traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/command/trajectory", 10);
    _traj_sampler_timer = nh.createTimer(ros::Duration(0.01), trajSamplerCallback); // 100 Hz evaluation

    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;


    for(int i = 0; i < time.size(); i++ )   // go through each segment
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }
    ROS_WARN("Trajectory length is %f m", traj_len);
    _wp_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_point";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);
      line_list.points.push_back(p);
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

VectorXd timeAllocation( MatrixXd Path)
{ 
    VectorXd time(Path.rows() - 1);

    // The time allocation is many relative timelines but not one common timeline
    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
        double x1 = _Vel * _Vel / (2 * _Acc); 
        double x2 = distance - 2 * x1;
        double t1 = _Vel / _Acc;
        double t2 = x2 / _Vel;
        time(i) = 2 * t1 + t2;
    }
    // cout << time << endl;

    return time;
}

Vector3d getVelPoly(MatrixXd polyCoeff, int k, double t)
{
    Vector3d ret;
    for (int dim = 0; dim < 3; dim++)
    {
        VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        VectorXd time = VectorXd::Zero(_poly_num1D);
        for(int j = 1; j < _poly_num1D; j++)
            time(j) = j * pow(t, j - 1);
        ret(dim) = coeff.dot(time);
    }
    return ret;
}

Vector3d getAccPoly(MatrixXd polyCoeff, int k, double t)
{
    Vector3d ret;
    for (int dim = 0; dim < 3; dim++)
    {
        VectorXd coeff = (polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        VectorXd time = VectorXd::Zero(_poly_num1D);
        for(int j = 2; j < _poly_num1D; j++)
            time(j) = j * (j - 1) * pow(t, j - 2);
        ret(dim) = coeff.dot(time);
    }
    return ret;
}

void trajSamplerCallback(const ros::TimerEvent& event)
{
    if (!_trajectory_active) return;

    double t_elapsed = (ros::Time::now() - _traj_start_time).toSec();
    int seg_idx = 0;
    double t_local = t_elapsed;

    // Determine which polynomial segment the drone is currently flying
    for (int i = 0; i < _polyTime.size(); i++) {
        if (t_local > _polyTime(i)) {
            t_local -= _polyTime(i);
            seg_idx++;
        } else {
            break;
        }
    }

    Vector3d p, v, a;
    if (seg_idx >= _polyTime.size()) {
        // Trajectory finished. Hover at the final waypoint.
        seg_idx = _polyTime.size() - 1;
        p = getPosPoly(_polyCoeff, seg_idx, _polyTime(seg_idx));
        v = Vector3d::Zero();
        a = Vector3d::Zero();
    } else {
        p = getPosPoly(_polyCoeff, seg_idx, t_local);
        v = getVelPoly(_polyCoeff, seg_idx, t_local);
        a = getAccPoly(_polyCoeff, seg_idx, t_local);
    }

    // 1. Cap velocity to a reasonable flight speed (e.g., 3.0 m/s)
    double max_vel = 3.0; 
    if (v.norm() > max_vel) {
        v = v.normalized() * max_vel;
    }

    // 2. Cap acceleration to prevent aggressive flips (e.g., 4.0 m/s^2)
    // 4.0 m/s^2 restricts the drone to roughly a 22-degree max tilt angle.
    double max_acc = 4.0; 
    if (a.norm() > max_acc) {
        a = a.normalized() * max_acc;
    }
    // ---------------------------------------------

    trajectory_msgs::MultiDOFJointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "map";

    trajectory_msgs::MultiDOFJointTrajectoryPoint pt;
    
    geometry_msgs::Transform transform;
    transform.translation.x = p(0);
    transform.translation.y = p(1);
    transform.translation.z = p(2);
    
    // Explicitly zero out the quaternion to prevent garbage memory from causing NaNs
    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = 0.0;
    transform.rotation.w = 1.0; 

    geometry_msgs::Twist vel;
    vel.linear.x = v(0); vel.linear.y = v(1); vel.linear.z = v(2);

    geometry_msgs::Twist acc;
    acc.linear.x = a(0); acc.linear.y = a(1); acc.linear.z = a(2);

    pt.transforms.push_back(transform);
    pt.velocities.push_back(vel);
    pt.accelerations.push_back(acc);

    traj_msg.points.push_back(pt);
    _cmd_traj_pub.publish(traj_msg);
}

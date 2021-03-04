/**
 * \brief This node is going to generate odometry message for a robot.
 * 
 * PUBLISHERS:
 * + odom (nav_msgs::Odometry) ~ odometry of robot
 * 
 * SUBSRIBERS:
 * + joint_states (sensor_msgs::JointState) ~ joint states of robot 
 * 
 * SERVICES:
 * + set_pose (rigid2d::SetPose) ~ reset odometry frame of robot
 * 
 * PARAMETERS:
 * + odom_frame_id (string) ~ name of the odometry tf frame
 * + body_frame_id (string) ~ name of the body tf frame
 * + left_wheel_joint (string) ~ name of the left wheel joint
 * + right_wheel_joint (string) ~ name of the right wheel joint
 * + wheel_base (double) ~ base dimension of robot wheel
 * + wheel_radius (double) ~ radius dimension of robot wheel
**/


#include <string>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "rigid2d/SetPose.h"
#include "rigid2d/SetPoseMsg.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuslam/kalman.hpp"

/// \brief Helper class for node odometer
class Handler {
  public:
      explicit Handler(ros::NodeHandle & n);

  private:
    //variables
    double wheel_base = 0.16;
    double wheel_radius = 0.033;
    std::string odom_frame_id;
    std::string body_frame_id;
    std::string left_wheel_joint;
    std::string right_wheel_joint;
    double noise_mean = 0.0;
    double noise_variance = 0.01;
    double slip_min = -0.05;
    double slip_max = 0.05;
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::vector<std::vector<double>> covariance_matrix = {{1.0, 1.0}, {1.0, 1.0}};;
    double obst_radius = 0.0762;
    double obst_max_dist = 2.0;
    double tube_var = 0.1;
    double rad_left_thresh = 0.0;
    double rad_right_thresh = 0.0;
    double rad_left_old = 0.0;
    double rad_right_old = 0.0;
    double rad_left_slip_old = 0.0;
    double rad_right_slip_old = 0.0;
    arma::vec init_map_state;
    rigid2d::DiffDrive odom;
    rigid2d::DiffDrive odom_slip;
    kalman::StateVec states;
    arma::vec world_map_tf = arma::vec {0, 0, 0};
    arma::vec measure_d;
    arma::vec measure_angle;
    arma::vec last_map_state;
    rigid2d::Transform2D T_odom_map {0};
    rigid2d::Transform2D T_tmp {0};
    // ROS members
    ros::Subscriber joint_states_sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster br;
    ros::ServiceServer set_pose_service;
    ros::Publisher odom_path_pub;
    nav_msgs::Path odom_path_msg; 
    ros::Publisher slam_path_pub;
    nav_msgs::Path slam_path_msg; 
    ros::Publisher slam_obst_pub;
    ros::Subscriber fake_measurenmt_sub;
    bool init = true;
    //helper functions
    void find_param(ros::NodeHandle & n);
    void joint_states_sub_callback(const sensor_msgs::JointState & msg);
    void tf_broadcast();
    bool set_pose_service_callback(rigid2d::SetPose::Request  & req, rigid2d::SetPose::Response & res);
    void pub_odom();
    void pub_slam_obst_marker();
    void align_tf();
    void fake_measurement_sub_callback(const std_msgs::Float64MultiArray & msg);
};

/// \brief Init Handler class
/// \param n - odometer NodeHandle
Handler::Handler(ros::NodeHandle & n) : odom(wheel_base/2, wheel_radius), \
        odom_slip(wheel_base/2, wheel_radius), states(init_map_state, noise_variance, tube_var) {
  joint_states_sub = n.subscribe("joint_states", 10, &Handler::joint_states_sub_callback, this);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  set_pose_service = n.advertiseService("set_pose", &Handler::set_pose_service_callback, this);
  odom_path_pub = n.advertise<nav_msgs::Path>("odom_path", 10);
  odom_path_msg.header.stamp=ros::Time::now();
  odom_path_msg.header.frame_id="world"; 
  slam_path_pub = n.advertise<nav_msgs::Path>("slam_path", 10);
  slam_path_msg.header.stamp=ros::Time::now();
  slam_path_msg.header.frame_id="world"; 
  slam_obst_pub = n.advertise<visualization_msgs::MarkerArray>( "slam_obst", 10 );
  fake_measurenmt_sub = n.subscribe("fake_measurement", 10, &Handler::fake_measurement_sub_callback, this);

  while (ros::ok()) {
    find_param(n);
    // ROS_INFO_STREAM("noises " << noise_variance << " " << tube_var);
    states.set_noises(noise_variance, tube_var);
    if (init) {
      last_map_state = init_map_state;
      states.set_map_state(init_map_state);

      measure_d = arma::vec (x_coords.size());
      measure_angle = arma::vec (x_coords.size());
      measure_d.fill(-1);
      measure_angle.fill(-1);

      init = false;
    }

    align_tf();
    pub_odom();
    pub_slam_obst_marker();
    tf_broadcast();

    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
  }
    
}

/// \brief publish odometry information to topic odom
void Handler::pub_odom() {
  // publish odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = odom_frame_id;
  odom_msg.child_frame_id = body_frame_id;
  rigid2d::Transform2D config = odom.config();
  odom_msg.pose.pose.position.x = config.x();
  odom_msg.pose.pose.position.y = config.y();
  odom_msg.pose.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, config.theta());
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();
  odom_pub.publish(odom_msg);

  // publish odometry path
  geometry_msgs::PoseStamped path_pose;
  path_pose.header.stamp = ros::Time::now();
  path_pose.header.frame_id = "odom";
  path_pose.pose.position.x = config.x();
  path_pose.pose.position.y = config.y();
  path_pose.pose.position.z = 0;
  path_pose.pose.orientation.x = q.x();
  path_pose.pose.orientation.y = q.y();
  path_pose.pose.orientation.z = q.z();
  path_pose.pose.orientation.w = q.w();

  odom_path_msg.poses.push_back(path_pose);
  odom_path_pub.publish(odom_path_msg);


  // pub path for turtle slam
  path_pose.header.stamp = ros::Time::now();
  // path_pose.header.frame_id = "slam_turtle";
  arma::vec robot_state = states.get_robot_state();
  rigid2d::Vector2D v {world_map_tf(1), world_map_tf(2)};
  rigid2d::Transform2D trans {v, world_map_tf(0)};
  rigid2d::Vector2D vr {robot_state(1), robot_state(2)};
  rigid2d::Transform2D trans_r {vr, robot_state(0)};
  rigid2d::Transform2D result = trans * trans_r;
  path_pose.pose.position.x =  result.x();
  path_pose.pose.position.y =  result.y();
  path_pose.pose.position.z = 0;
  q.setRPY(0, 0,  result.theta());
  path_pose.pose.orientation.x = q.x();
  path_pose.pose.orientation.y = q.y();
  path_pose.pose.orientation.z = q.z();
  path_pose.pose.orientation.w = q.w();

  slam_path_msg.poses.push_back(path_pose);
  slam_path_pub.publish(slam_path_msg);
}

/// \brief Find parameters odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint, 
/// wheel_base, and wheel_radius from ROS parameter server, otherwise set to default value
/// \param n - odometer NodeHandle
void Handler::find_param(ros::NodeHandle & n) {
  std::string default_odom = "odom";
  std::string default_base = "base_footprint";
  std::string default_left = "wheel_left_link";
  std::string default_right = "wheel_right_link";
  std::vector<double> default_x_coords = {0.0, 1.0, 2.0};
  std::vector<double> default_y_coords = {0.0, 1.0, 2.0};
  std::vector<std::vector<double>> default_cm = {{1.0, 1.0}, {1.0, 1.0}};
  n.param("wheel_base", wheel_base, 0.16);
  n.param("wheel_radius", wheel_radius, 0.033);
  n.param("odom_frame_id", odom_frame_id, default_odom);
  n.param("body_frame_id", body_frame_id, default_base);
  n.param("left_wheel_joint", left_wheel_joint, default_left);
  n.param("right_wheel_joint", right_wheel_joint, default_right);
  n.param("noise_mean", noise_mean, 0.0);
  n.param("noise_variance", noise_variance, 0.01);
  n.param("slip_min", slip_min, -0.05);
  n.param("slip_max", slip_max, 0.05);
  n.param("x_coords", x_coords, default_x_coords);
  n.param("y_coords", y_coords, default_y_coords);
  n.param("covariance_matrix_0", covariance_matrix[0], default_cm[0]);
  n.param("covariance_matrix_1", covariance_matrix[1], default_cm[1]);
  n.param("obst_radius", obst_radius, 0.0762);
  n.param("obst_max_dist", obst_max_dist, 2.0);
  n.param("tube_var", tube_var, 0.01);

  init_map_state = arma::vec (x_coords.size()*2);
  for (unsigned i = 0; i < x_coords.size(); i++) {
    init_map_state(2*i) = x_coords[i];
    init_map_state(2*i+1) = y_coords[i];
  }

}

/// \brief Callback function for joint_states subscriber.
/// Updates robot config given updated wheel angles.
/// \param msg - message from subscriber
void Handler::joint_states_sub_callback(const sensor_msgs::JointState & msg) {
  double rad_left = msg.position[0] - rad_left_thresh;
  double rad_right = msg.position[1] - rad_right_thresh;
  double rad_left_slip = msg.position[2] - rad_left_thresh;
  double rad_right_slip = msg.position[3] - rad_right_thresh;

  if (rad_left != 0 && rad_right != 0) {
    double rad_left_diff = rad_left - rad_left_old;
    rad_left_diff = rigid2d::normalize_angle(rad_left_diff);
    double rad_right_diff = rad_right - rad_right_old;
    rad_right_diff = rigid2d::normalize_angle(rad_right_diff);
    odom.update(rad_left_diff, rad_right_diff);
    rad_left_old = rad_left;
    rad_right_old = rad_right;

    double rad_left_diff_slip = rad_left_slip - rad_left_slip_old;
    rad_left_diff_slip = rigid2d::normalize_angle(rad_left_diff_slip);
    double rad_right_diff_slip = rad_right_slip - rad_right_slip_old;
    rad_right_diff_slip = rigid2d::normalize_angle(rad_right_diff_slip);
    odom_slip.update(rad_left_diff_slip, rad_right_diff_slip);
    rad_left_slip_old = rad_left_slip;
    rad_right_slip_old = rad_right_slip;

    // ROS_INFO_STREAM("Incoming " << measure_d.t() << " " << measure_angle.t());
    rigid2d::DiffDriveVel vel {rad_left_diff, rad_right_diff};
    rigid2d::Twist2D t = odom_slip.twist_from_vel(vel);
    // ROS_INFO_STREAM("Calculated twist " << t);
    last_map_state = states.get_map_state();
    // ROS_INFO_STREAM("store map " << last_map_state);
    states.ekf_update(t, measure_d, measure_angle);
    
    // arma::vec d {sqrt(1+0.5*0.5), sqrt(8), sqrt(18)};
    // arma::vec ang {atan2(0.5,1), atan(1), atan(1)};
    // states.ekf_update(t, d, ang);

    // rigid2d::Transform2D config = odom.config();
    // rigid2d::Transform2D config_slip = odom_slip.config();
    // ROS_INFO_STREAM("updated odom: " << config.theta() << " "<<config.x()<<" " << config.y() << \
    //                 " -VS- updated slip " <<  config_slip.theta() << " "<<config_slip.x()<<" " << config_slip.y());
  }
}


/// \brief Broadcast the transform between odom_frame_id and the body_frame_id
void Handler::tf_broadcast() {
  // tf from odom to base_foot_print
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = odom_frame_id;
  transformStamped.child_frame_id = body_frame_id;
  rigid2d::Transform2D config = odom.config();
  transformStamped.transform.translation.x = config.x();
  transformStamped.transform.translation.y = config.y();
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, config.theta());
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);

  // tf from world to map
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "map";
  transformStamped.transform.translation.x = world_map_tf(1);
  transformStamped.transform.translation.y = world_map_tf(2);
  transformStamped.transform.translation.z = 0.0;
  q.setRPY(0, 0,  world_map_tf(0));
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);


  // tf from map to slam
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "slam_turtle";
  arma::vec robot_state = states.get_robot_state();
  // ROS_INFO_STREAM("map state = " << robot_state);
  transformStamped.transform.translation.x = robot_state(1);
  transformStamped.transform.translation.y = robot_state(2);
  transformStamped.transform.translation.z = 0.0;
  q.setRPY(0, 0, robot_state(0));
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);


}

/// \brief Callback function for set_pose service 
/// \param req - request of set_pose service 
/// \param res - responce of set_pose service 
/// \returns true if service set_pose is success and complete
bool Handler::set_pose_service_callback(rigid2d::SetPose::Request  & req, rigid2d::SetPose::Response & res) {
  double theta = req.msg.theta;
  double x = req.msg.x;
  double y = req.msg.y;
  odom.set_pose(theta, x, y);
  return true;
}




/// \brief Helper function for publishing obstacle markers
void Handler::pub_slam_obst_marker() {
  visualization_msgs::MarkerArray marker_array;
  arma::vec map_state = states.get_map_state();
  // ROS_INFO_STREAM("map states " << map_state);
  for (unsigned i = 0; i < map_state.size()/2; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "slam_obst";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = map_state[2*i];
    marker.pose.position.y = map_state[2*i+1];
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = obst_radius*2;
    marker.scale.y = obst_radius*2;
    marker.scale.z = 1.5;
    marker.color.a = 0.5; 
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);
  }
  slam_obst_pub.publish( marker_array );
}

void Handler::align_tf() {
  arma::vec map_state = states.get_map_state();
  arma::vec robot_state = states.get_robot_state();
  // ROS_INFO_STREAM("current map " << map_state);
  rigid2d::Vector2D v {robot_state(1), robot_state(2)};
  rigid2d::Transform2D T_map_robot { v, robot_state(0)};
  rigid2d::Transform2D config = odom_slip.config();

  world_map_tf = {0, 0, 0};
  T_odom_map = config * T_map_robot.inv();


  ROS_INFO_STREAM("T_odom_map = " << T_odom_map);
  unsigned n = 0;
  for (unsigned i = 0; i < x_coords.size()-1; i++) {
    if (measure_d(i) > 0) {
      n += 1;
      // direction of obstacles 
      double mx = map_state(2*i) - map_state(2*(i+1));
      double my = map_state(2*i+1) - map_state(2*(i+1)+1);
      double map_ang = atan2(my, mx);

      double ox = x_coords[i] - x_coords[i+1];
      double oy = y_coords[i] - y_coords[i+1];
      double odom_ang = atan2(oy, ox);

      // get translation 
      rigid2d::Vector2D v_map {map_state(0), map_state(1)};
      double theta_map = atan2(map_state(1), map_state(0));
      rigid2d::Vector2D v_odom {x_coords[0], y_coords[0]};
      double theta_odom = atan2(y_coords[0], x_coords[0]);

      rigid2d::Transform2D Tm {v_map, map_ang};
      rigid2d::Transform2D To {v_odom, odom_ang};
      rigid2d::Transform2D result =  To * Tm.inv();

      world_map_tf(0) += result.theta();
      world_map_tf(1) += result.x();
      world_map_tf(2) += result.y();

    }
  }
  world_map_tf(0) /= n;
  world_map_tf(1) /= n;
  world_map_tf(2) /= n;

  ROS_INFO_STREAM(" === Final tf = " << world_map_tf.t());

    
}

void Handler::fake_measurement_sub_callback(const std_msgs::Float64MultiArray & msg) {
  arma::vec md_tmp (msg.data.size()/2);
  arma::vec ma_tmp (msg.data.size()/2);
  for (int i = 0; i < x_coords.size(); i++) {
    md_tmp(i) = msg.data[2*i];
    ma_tmp(i) = msg.data[2*i+1];
    ma_tmp(i) = ma_tmp(i);
  }
  measure_d = md_tmp;
  measure_angle = ma_tmp;
}





int main(int argc, char **argv) {
  ros::init(argc, argv, "odometer");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
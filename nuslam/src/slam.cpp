/**
 * \brief This node is going to calculate robot configuration and obstacle locations by inputting
 * commanded twist and sensor information into extended Kalman Filter SLAM algorithm. 
 * 
 * 
 * PUBLISHERS:
 * + odom (nav_msgs::Odometry) ~ odometry of robot
 * + odom_path (nav_msgs::Path) ~ path of odometry turtle 
 * + slam_path (nav_msgs::Path) ~ path of turtle config calculated from Kalman Filter Slam
 * + slam_obst (<visualization_msgs::MarkerArray) ~ obstacle locations calculated from Kalman Filter Slam
 * 
 * SUBSRIBERS:
 * + joint_states (sensor_msgs::JointState) ~ joint states of robot 
 * + fake_measurement (std_msgs::Float64MultiArray) ~ fake sensor information containing relative distance and angle to obstacles 
 * 
 * SERVICES:
 * + set_pose (rigid2d::SetPose) ~ reset odometry frame of robot
 * 
 * PARAMETERS:
 * + wheel_base (string) ~ distance between wheels of robot  
 * + wheel_radius (string) ~ radius of wheels  
 * + odom_frame_id (string) ~ name of the odometry tf frame
 * + body_frame_id (string) ~ name of the body tf frame
 * + left_wheel_joint (string) ~ name of robot left wheel joint  
 * + right_wheel_joint (string) ~ name of robot right wheel joint  
 * + left_wheel_joint_slip (string) ~ name of robot left wheel joint with noises  
 * + right_wheel_joint_slip (string) ~ name of robot right wheel joint with noises  
 * + x_coords (double list) ~ x locations of obstacles in world  
 * + y_coords (double list) ~ y locations of obstacles in world  
 * + covariance_matrix_0 (double list) ~ first row of covariance matrix of obstacle lcoations   
 * + covariance_matrix_1 (double list)~ second row of covariance matrix of obstacle lcoations  
 * + obst_radius (double) ~ radius of tube obstacle   
 * + state_var (double) ~ variance of Gaussian noise for Kalman states   
 * + sensor_var (double) ~ variance of Gaussian noise for sensors   
 * 
 * 
 * 
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

/// \brief Helper class for node slam
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
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::vector<std::vector<double>> covariance_matrix = {{1.0, 1.0}, {1.0, 1.0}};;
    double obst_radius = 0.0762;
    double rad_left_thresh = 0.0;
    double rad_right_thresh = 0.0;
    double rad_left_old = 0.0;
    double rad_right_old = 0.0;
    double rad_left_slip_old = 0.0;
    double rad_right_slip_old = 0.0;
    double state_var = 0.01;
    double sensor_var = 0.001;
    arma::vec seen;
    arma::vec init_map_state;
    rigid2d::DiffDrive odom;
    rigid2d::DiffDrive odom_slip;
    kalman::StateVec states;
    arma::vec map_odom_tf = arma::vec {0, 0, 0};
    arma::vec measure_d;
    arma::vec measure_angle;
    arma::vec last_map_state;
    rigid2d::Transform2D T_odom_map {0};
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
/// \param n - slam NodeHandle
Handler::Handler(ros::NodeHandle & n) : odom(wheel_base/2, wheel_radius), \
              odom_slip(wheel_base/2, wheel_radius), states(init_map_state, state_var, sensor_var) {
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

    // set noise variance for Kalman slam
    states.set_noises(state_var, sensor_var);

    // initialize Kalman slam
    if (init) {
      last_map_state = init_map_state;
      seen = arma::vec (init_map_state.size()/2);
      seen.fill(false);
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

/// \brief Publish odometry information to topic odom and path of odometry.
/// This function also publishes turtle path calcualted from Kalman slam
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


  // publish slam turtle path
  path_pose.header.stamp = ros::Time::now();
  arma::vec robot_state = states.get_robot_state();
  rigid2d::Vector2D v {map_odom_tf(1), map_odom_tf(2)};
  rigid2d::Transform2D trans {v, map_odom_tf(0)};
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

/// \brief Find parameters
/// \param n - slam NodeHandle
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
  n.param("x_coords", x_coords, default_x_coords);
  n.param("y_coords", y_coords, default_y_coords);
  n.param("covariance_matrix_0", covariance_matrix[0], default_cm[0]);
  n.param("covariance_matrix_1", covariance_matrix[1], default_cm[1]);
  n.param("obst_radius", obst_radius, 0.0762);
  n.param("state_var", state_var, 0.01);
  n.param("sensor_var", sensor_var, 0.001);


  init_map_state = arma::vec (x_coords.size()*2);
  for (unsigned i = 0; i < x_coords.size(); i++) {
    init_map_state(2*i) = x_coords[i];
    init_map_state(2*i+1) = y_coords[i];
  }

}

/// \brief Callback function for joint_states subscriber.
/// Updates robot config given updated wheel angles.
/// Also updates robot config with noise and collision for checking correctness of Kalman slam.
/// After calcualting twist from joint states, input twist and fake measurement into
/// Kalman Filter to calcualte slam turlte pose and slam obstacle locations.
/// \param msg - message from subscriber
void Handler::joint_states_sub_callback(const sensor_msgs::JointState & msg) {
  double rad_left = msg.position[0] - rad_left_thresh;
  double rad_right = msg.position[1] - rad_right_thresh;
  double rad_left_slip = msg.position[2] - rad_left_thresh;
  double rad_right_slip = msg.position[3] - rad_right_thresh;

  if (rad_left != 0 && rad_right != 0) {
    // odom turtle 
    double rad_left_diff = rad_left - rad_left_old;
    rad_left_diff = rigid2d::normalize_angle(rad_left_diff);
    double rad_right_diff = rad_right - rad_right_old;
    rad_right_diff = rigid2d::normalize_angle(rad_right_diff);
    odom.update(rad_left_diff, rad_right_diff);
    rad_left_old = rad_left;
    rad_right_old = rad_right;

    // turtle with noise
    double rad_left_diff_slip = rad_left_slip - rad_left_slip_old;
    rad_left_diff_slip = rigid2d::normalize_angle(rad_left_diff_slip);
    double rad_right_diff_slip = rad_right_slip - rad_right_slip_old;
    rad_right_diff_slip = rigid2d::normalize_angle(rad_right_diff_slip);
    odom_slip.update(rad_left_diff_slip, rad_right_diff_slip);
    rad_left_slip_old = rad_left_slip;
    rad_right_slip_old = rad_right_slip;

    // calcualte slam turtle 
    rigid2d::DiffDriveVel vel {rad_left_diff, rad_right_diff};
    rigid2d::Twist2D t = odom_slip.twist_from_vel(vel);
    last_map_state = states.get_map_state();
    states.ekf_update(t, measure_d, measure_angle);
  }
}


/// \brief Broadcast the transform between frames:
///     odom -> fake turtle 
///     map -> odom
///     odom -> slam turtle
///     odom -> base_footprint
/// After transform, world -> map -> odom -> base_footprint should let
/// a robot model have a similar pose as turtle with noise.
void Handler::tf_broadcast() {
  // odom -> odometry turtle 
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "fake_turtle";
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

  // map -> odom
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "odom";
  transformStamped.transform.translation.x = map_odom_tf(1);
  transformStamped.transform.translation.y = map_odom_tf(2);
  transformStamped.transform.translation.z = 0.0;
  q.setRPY(0, 0,  map_odom_tf(0));
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);

  // odom -> slam turtle
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "slam_turtle";
  arma::vec robot_state = states.get_robot_state();
  transformStamped.transform.translation.x = robot_state(1);
  transformStamped.transform.translation.y = robot_state(2);
  transformStamped.transform.translation.z = 0.0;
  q.setRPY(0, 0, robot_state(0));
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);

  // odom -> base_footprint
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = body_frame_id;
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

/// \brief Helper function for publishing obstacle marker locations calculated 
/// from Kalman slam. Only obstacles that are seen before are published. 
void Handler::pub_slam_obst_marker() {
  visualization_msgs::MarkerArray marker_array;
  arma::vec map_state = states.get_map_state();
  for (unsigned i = 0; i < map_state.size()/2; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "slam_obst";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    if (seen(i) == true) {
      marker.action = visualization_msgs::Marker::ADD;
    }
    else {
      marker.action = visualization_msgs::Marker::DELETE;
    }
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

/// \brief Compute the transformation between map and odom
/// The T_map_odom should be the same as T_first_second_obst
/// If only one obstacle is seen, T_odom_map is calculated from T_first_obst_map.
/// However, in this case, the angle of transformation may be incorrect. 
void Handler::align_tf() {
  arma::vec map_state = states.get_map_state();
  arma::vec robot_state = states.get_robot_state();
  // ROS_INFO_STREAM("current map " << map_state);
  rigid2d::Vector2D v {robot_state(1), robot_state(2)};
  rigid2d::Transform2D T_map_robot { v, robot_state(0)};
  rigid2d::Transform2D config = odom_slip.config();

  map_odom_tf = {0, 0, 0};
  map_odom_tf.fill(0);

  // T_odom_map = config * T_map_robot.inv();
  // ROS_INFO_STREAM("T_odom_map = " << T_odom_map.inv());

  unsigned n = 0;
  for (unsigned i = 0; i < x_coords.size()-1; i++) {
    unsigned j = i+1;
    for (j; j < x_coords.size(); j++) {
      if (measure_d(i) > 0 && measure_d(j) > 0) {
        n += 1;

        double mx, my, ox, oy;
        mx = map_state(2*i) - map_state(2*j);
        my = map_state(2*i+1) - map_state(2*j+1);
        ox = x_coords[i] - x_coords[j];
        oy = y_coords[i] - y_coords[j];
              
        // direction of obstacles 
        double map_ang = atan2(my, mx);
        double odom_ang = atan2(oy, ox);

        // get translation 
        rigid2d::Vector2D v_map {map_state(0), map_state(1)};
        double theta_map = atan2(map_state(1), map_state(0));
        rigid2d::Vector2D v_odom {x_coords[0], y_coords[0]};
        double theta_odom = atan2(y_coords[0], x_coords[0]);

        rigid2d::Transform2D Tm {v_map, map_ang};
        rigid2d::Transform2D To {v_odom, odom_ang};
        rigid2d::Transform2D result =  To * Tm.inv();

        map_odom_tf(0) += result.theta();
        map_odom_tf(1) += result.x();
        map_odom_tf(2) += result.y();
      }
    }

    if (i == 0 && j >= x_coords.size()) { // only one obstacle
        n = 1;

        double mx, my, ox, oy;
        mx = map_state(2*i);
        my = map_state(2*i+1);
        ox = x_coords[i];
        oy = y_coords[i];
              
        // direction of obstacles 
        double map_ang = atan2(my, mx);
        double odom_ang = atan2(oy, ox);

        // get translation 
        rigid2d::Vector2D v_map {map_state(0), map_state(1)};
        double theta_map = atan2(map_state(1), map_state(0));
        rigid2d::Vector2D v_odom {x_coords[0], y_coords[0]};
        double theta_odom = atan2(y_coords[0], x_coords[0]);

        rigid2d::Transform2D Tm {v_map, map_ang};
        rigid2d::Transform2D To {v_odom, odom_ang};
        rigid2d::Transform2D result =  To * Tm.inv();

        map_odom_tf(0) = result.theta();
        map_odom_tf(1) = result.x();
        map_odom_tf(2) = result.y();
    }
  }
  // average transform
  if (n > 0) {
    map_odom_tf(0) /= n;
    map_odom_tf(1) /= n;
    map_odom_tf(2) /= n;
  }
  // ROS_INFO_STREAM(" === Final tf = " << map_odom_tf.t());
}

/// \brief Callback function for fake_measurement subscriber
/// \param msg - float array in form {d1, theat_1, d2, theta_2, ...}
///               where d is the distance theta is the realtive angle to obstacles. 
/// -1 indicates obstacles that are not seen.  
void Handler::fake_measurement_sub_callback(const std_msgs::Float64MultiArray & msg) {
  arma::vec md_tmp (msg.data.size()/2);
  arma::vec ma_tmp (msg.data.size()/2);
  for (int i = 0; i < x_coords.size(); i++) {
    md_tmp(i) = msg.data[2*i];
    ma_tmp(i) = msg.data[2*i+1];
    ma_tmp(i) = ma_tmp(i);
    if (md_tmp(i) > 0 && seen(i) == false) {
      seen(i) = true;
    } 
  }
  measure_d = md_tmp;
  measure_angle = ma_tmp;
  // ROS_INFO_STREAM("Check map " << seen.t() << " vs " << measure_d.t());
}





int main(int argc, char **argv) {
  ros::init(argc, argv, "slam");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
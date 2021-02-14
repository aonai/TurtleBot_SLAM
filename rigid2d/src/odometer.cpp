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
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "rigid2d/SetPose.h"
#include "rigid2d/SetPoseMsg.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

/// \brief Helper class for node odometer
class Handler {
  public:
      explicit Handler(ros::NodeHandle & n);

  private:
    //variables
    std::string odom_frame_id;
    std::string body_frame_id;
    std::string left_wheel_joint;
    std::string right_wheel_joint;
    double wheel_base = 0.16;
    double wheel_radius = 0.033;
    double rad_left_thresh = 0.0;
    double rad_right_thresh = 0.0;
    double rad_left_old = 0.0;
    double rad_right_old = 0.0;
    rigid2d::DiffDrive odom;
    // ROS members
    ros::Subscriber joint_states_sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster br;
    ros::ServiceServer set_pose_service;
    //helper functions
    void find_param(ros::NodeHandle & n);
    void joint_states_sub_callback(const sensor_msgs::JointState & msg);
    void tf_broadcast();
    bool set_pose_service_callback(rigid2d::SetPose::Request  & req, rigid2d::SetPose::Response & res);
    void pub_odom();
};

/// \brief Init Handler class
/// \param n - odometer NodeHandle
Handler::Handler(ros::NodeHandle & n) : odom(wheel_base/2, wheel_radius) {
  joint_states_sub = n.subscribe("joint_states", 10, &Handler::joint_states_sub_callback, this);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  set_pose_service = n.advertiseService("set_pose", &Handler::set_pose_service_callback, this);

  while (ros::ok()) {
    find_param(n);

    pub_odom();
    tf_broadcast();
    
    ros::Rate loop_rate(50);
    ros::spinOnce();
    loop_rate.sleep();
  }
    
}

/// \brief publish odometry information to topic odom
void Handler::pub_odom() {
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
}

/// \brief Find parameters odom_frame_id, body_frame_id, left_wheel_joint, right_wheel_joint, 
/// wheel_base, and wheel_radius from ROS parameter server, otherwise set to default value
/// \param n - odometer NodeHandle
void Handler::find_param(ros::NodeHandle & n) {
  std::string default_odom = "odom";
  std::string default_base = "base_footprint";
  std::string default_left = "wheel_left_link";
  std::string default_right = "wheel_right_link";
  n.param("odom_frame_id", odom_frame_id, default_odom);
  n.param("body_frame_id", body_frame_id, default_base);
  n.param("left_wheel_joint", left_wheel_joint, default_left);
  n.param("right_wheel_joint", right_wheel_joint, default_right);
  n.param("wheel_base", wheel_base, 0.16);
  n.param("wheel_radius", wheel_radius, 0.033);
}

/// \brief Callback function for joint_states subscriber.
/// Updates robot config given updated wheel angles.
/// \param msg - message from subscriber
void Handler::joint_states_sub_callback(const sensor_msgs::JointState & msg) {
  double rad_left = msg.position[0] - rad_left_thresh;
  double rad_right = msg.position[1] - rad_right_thresh;

  if (rad_left != 0 && rad_right != 0) {
    double rad_left_diff = rad_left - rad_left_old;
    rad_left_diff = rigid2d::normalize_angle(rad_left_diff);
    double rad_right_diff = rad_right - rad_right_old;
    rad_right_diff = rigid2d::normalize_angle(rad_right_diff);
    odom.update(rad_left_diff, rad_right_diff);
    rad_left_old = rad_left;
    rad_right_old = rad_right;
  }
  rigid2d::Transform2D config = odom.config();
  // ROS_INFO_STREAM("updated odom: " << config.theta() << " "<<config.x()<<" " << config.y());
}


/// \brief Broadcast the transform between odom_frame_id and the body_frame_id
void Handler::tf_broadcast() {
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



int main(int argc, char **argv) {
  ros::init(argc, argv, "odometer");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
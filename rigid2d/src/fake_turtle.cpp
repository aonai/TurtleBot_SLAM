/**
 * \brief This node is going to generate a fake turtle that is controlled by /cmd_vel and 
 * publishes joint states.
 * 
 * PUBLISHERS:
 * + joint_states (sensor_msgs::JointState) ~ joint states of turtle
 *   
 * SUBSRIBERS:
 * + cmd_vel (geometry_msgs::Twist) ~ the velocity of turtle
 *  
 * PARAMETERS:
 * + left_wheel_joint (string) ~ name of the left wheel joint
 * + right_wheel_joint (string) ~ name of the right wheel joint
 * + wheel_base (double) ~ base dimension of robot wheel
 * + wheel_radius (double) ~ radius dimension of robot wheel
 * 
**/


#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"


/// \brief Helper class for node fake_turtle
class Handler {
  public:
    explicit Handler(ros::NodeHandle & n);

  private:
    // variables 
    double wheel_base = 0.16;
    double wheel_radius = 0.033;
    std::string left_wheel_joint;
    std::string right_wheel_joint;
    double current_rad_left = 0;
    double current_rad_right = 0;
    rigid2d::DiffDrive fake;             
    ros::Time begin = ros::Time::now();
    // ROS members
    ros::Subscriber cmd_vel_sub;
    ros::Publisher joint_state_pub;
    // helper functions
    void find_param(ros::NodeHandle & n);
    void cmd_vel_sub_callback(const geometry_msgs::Twist & vel);
    void pub_joint_state();

};

/// \brief Init Handler class
/// \param n - fake_turtle NodeHandle
Handler::Handler(ros::NodeHandle & n) : fake(wheel_base/2, wheel_radius) {
  cmd_vel_sub = n.subscribe("cmd_vel", 10, &Handler::cmd_vel_sub_callback, this);
  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    
  while (ros::ok()) {
    find_param(n);
    
    pub_joint_state();

    ros::Rate loop_rate(100);
    ros::spinOnce();
    loop_rate.sleep();
  }
    
}

/// \brief Find parameters wheel_base, and wheel_radius, left_wheel_joint, and right_wheel_joint
/// from ROS parameter server, otherwise set to default value
/// \param n - fake_turtle NodeHandle
void Handler::find_param(ros::NodeHandle & n) {
  std::string default_left = "wheel_left_link";
  std::string default_right = "wheel_right_link";
  n.param("left_wheel_joint", left_wheel_joint, default_left);
  n.param("right_wheel_joint", right_wheel_joint, default_right);
  n.param("wheel_base", wheel_base, 0.16);
  n.param("wheel_radius", wheel_radius, 0.033);
}

/// \brief Callback function for cmd_vel subscriber
/// Compute wheel velocities from given twist (cmd_vel), then update turtle config
/// and wheel angles. 
/// \param vel - velocity of turtle
void Handler::cmd_vel_sub_callback(const geometry_msgs::Twist  & vel) {
  ros::Duration period = ros::Time::now() - begin;
  double period_secs = period.toSec();
   if (period_secs > 1e-3) {
      begin = ros::Time::now();
      double omg = vel.angular.z*period_secs;
      double vx = vel.linear.x*period_secs;

      rigid2d::Twist2D t {omg, vx, 0};
      rigid2d::DiffDriveVel wheel_vel = fake.vel_from_twist(t);
      fake.update(wheel_vel.vL, wheel_vel.vR);

      current_rad_left += wheel_vel.vL;
      current_rad_right += wheel_vel.vR;

      rigid2d::Transform2D config = fake.config();
      ROS_INFO_STREAM("updated fake: " << config.theta() << config.x() << config.y() << " period = " << period_secs);
   }
}

/// \brief Helper function for publishing joint states
void Handler::pub_joint_state() {
  sensor_msgs::JointState js;
  std::vector<std::string> name = {left_wheel_joint, right_wheel_joint};
  std::vector<double> pos = {current_rad_left, current_rad_right};
  js.name = name;
  js.position = pos;
  joint_state_pub.publish(js);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_turtle");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
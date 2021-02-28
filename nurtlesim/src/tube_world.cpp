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
#include<random>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"


/// \brief Helper class for node tube_world
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
    double noise_mean = 0.0;
    double noise_variance = 0.01;
    double slip_min = -0.05;
    double slip_max = 0.05;
    rigid2d::DiffDrive fake;             
    rigid2d::DiffDrive fake_slip;             
    ros::Time begin = ros::Time::now();
    // ROS members
    ros::Subscriber cmd_vel_sub;
    ros::Publisher joint_state_pub;
    // helper functions
    void find_param(ros::NodeHandle & n);
    void cmd_vel_sub_callback(const geometry_msgs::Twist & vel);
    void pub_joint_state();
    std::mt19937 & get_random();
};

/// \brief Init Handler class
/// \param n - tube_world NodeHandle
Handler::Handler(ros::NodeHandle & n) : fake(wheel_base/2, wheel_radius), fake_slip(wheel_base/2, wheel_radius) {
  cmd_vel_sub = n.subscribe("cmd_vel", 10, &Handler::cmd_vel_sub_callback, this);
  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
    
  while (ros::ok()) {
    find_param(n);
    
    pub_joint_state();

    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
  }
    
}

/// \brief Helper function for generating random gaussian noise
/// \return A random generator 
std::mt19937 & Handler::get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

/// \brief Find parameters wheel_base, and wheel_radius, left_wheel_joint, and right_wheel_joint
/// from ROS parameter server, otherwise set to default value
/// \param n - tube_world NodeHandle
void Handler::find_param(ros::NodeHandle & n) {
  std::string default_left = "wheel_left_link";
  std::string default_right = "wheel_right_link";
  n.param("wheel_base", wheel_base, 0.16);
  n.param("wheel_radius", wheel_radius, 0.033);
  n.param("left_wheel_joint", left_wheel_joint, default_left);
  n.param("right_wheel_joint", right_wheel_joint, default_right);
  n.param("noise_mean", noise_mean, 0.0);
  n.param("noise_variance", noise_variance, 0.01);
  n.param("slip_min", slip_min, -0.05);
  n.param("slip_max", slip_max, 0.05);
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
      double omg = vel.angular.z;
      double vx = vel.linear.x;

      // add gaussian noise 
      std::normal_distribution<> d(noise_mean, noise_variance);
      if (omg != 0) {
        double noise_omg = d(get_random());
        omg = omg + noise_omg;
        // ROS_INFO_STREAM("omg with noise: " << omg << " from: " << noise_omg );
      }
      if (vx != 0) {
        double noise_vx = d(get_random());
        vx = vx + noise_vx;
        // ROS_INFO_STREAM("vx with noise: " << vx << " from: " << noise_vx);
      }

      omg = omg*period_secs;
      vx = vx*period_secs;

      rigid2d::Twist2D t {omg, vx, 0};
      rigid2d::DiffDriveVel wheel_vel = fake.vel_from_twist(t);
      fake.update(wheel_vel.vL, wheel_vel.vR);

      std::uniform_real_distribution<double> d_slip(slip_min, slip_max);
      double vL_slip = wheel_vel.vL;
      double vR_slip = wheel_vel.vR;
      // ROS_INFO_STREAM("slip noise: " << noise_slip);
      if (vL_slip != 0) {
        vL_slip *= d_slip(get_random()) + 1;
      }      
      if (vR_slip != 0) {
        vR_slip *= d_slip(get_random()) + 1;
      }
      fake_slip.update(vL_slip, vR_slip);

      current_rad_left += vL_slip;
      current_rad_right += vR_slip;

      // ROS_INFO_STREAM("Add slipping " << wheel_vel.vL << " " << wheel_vel.vR << " to " << vL_slip << " " << vR_slip );

      rigid2d::Transform2D config = fake.config();
      ROS_INFO_STREAM("updated fake: " << config.theta() << " " << config.x() << " " << config.y());

      rigid2d::Transform2D config_slip = fake_slip.config();
      ROS_INFO_STREAM("updated fake_slip: " << config_slip.theta() << " " << config_slip.x() << " " << config_slip.y());
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
  ros::init(argc, argv, "tube_world");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
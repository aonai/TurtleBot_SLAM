/**
 * \brief This node is going to test node turtle_interface using catch_ros
 * 
 * PUBLISHERS:
 * + cmd_vel (geometry_msgs::Twist) ~ velocity of turtle in form of twist
 * + sensor_data (nuturtlebot::SensorData) ~ sensor data of turtle
 * 
 * SUBSRIBERS:
 * + joint_states (sensor_msgs::JointState) ~ joint states of robot 
 * + wheel_cmd (nuturtlebot::WheelCommands) ~ left and right wheel velocities of turtle
 * 
**/

#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rigid2d/diff_drive.hpp"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "sensor_msgs/JointState.h"
#include <catch_ros/catch.hpp>


/// \brief Helper class for testing turtle_interface
class Handler {
  public:
    explicit Handler(ros::NodeHandle & n);
    double left_vel = -1;
    double right_vel = -1;     
    double left_vel_js = -1; 
    double right_vel_js = -1; 
    // helper functions
    void pub_cmd_vel(double vx, double omg);
    void pub_sensor_data(int left_encoder, int right_encoder);

  private:
    ros::Publisher cmd_vel_pub;
    ros::Publisher sensor_data_pub;
    ros::Subscriber wheel_cmd_sub;
    ros::Subscriber joint_states_sub;
    void wheel_cmd_sub_callback(const nuturtlebot::WheelCommands & vel);
    void joint_states_sub_callback(const sensor_msgs::JointState & js);
};

/// \brief Init Handler class
/// \param n - turtle_interface_test NodeHandle
Handler::Handler(ros::NodeHandle & n) {
  cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  sensor_data_pub = n.advertise<nuturtlebot::SensorData>("sensor_data", 10);
  wheel_cmd_sub = n.subscribe("wheel_cmd", 10, &Handler::wheel_cmd_sub_callback, this);
  joint_states_sub = n.subscribe("joint_states", 10, &Handler::joint_states_sub_callback, this);
}


/// \brief Callback function for wheel_cmd subscriber
/// \param vel - wheel velocities of turtle
void Handler::wheel_cmd_sub_callback(const nuturtlebot::WheelCommands & vel) {
  left_vel = vel.left_velocity;
  right_vel = vel.right_velocity;
  // ROS_INFO_STREAM("received wheel vel: " << vel.left_velocity <<" "<< vel.right_velocity);
}

/// \brief Callback function for joint_states subscriber
/// \param js - joint states of turtle
void Handler::joint_states_sub_callback(const sensor_msgs::JointState & js) {
  left_vel_js = js.velocity[0];
  right_vel_js = js.velocity[1];
  // ROS_INFO_STREAM("received js vel: " << left_vel_js << " " << right_vel_js);
}

/// \brief Helper function for pubslihing cmd_vel
void Handler::pub_cmd_vel(double vx, double omg) {
  geometry_msgs::Twist msg;
  msg.linear.x = vx;
  msg.angular.z = omg;
  cmd_vel_pub.publish(msg);
}

/// \brief Helper function for pubslihing sensor data
void Handler::pub_sensor_data(int left_encoder, int right_encoder) {
  nuturtlebot::SensorData msg;
  msg.left_encoder = left_encoder;
  msg.right_encoder = right_encoder;
  sensor_data_pub.publish(msg);
}


TEST_CASE( "Test velocity commands", "[cmd_vel, wheel_cmd]" ) {
  ros::NodeHandle n;
  Handler tester(n);
 
  SECTION( "pure translation" ) {
    ros::Time begin = ros::Time::now();
    while (ros::ok()) {
      tester.pub_cmd_vel(1, 0);

      ros::Rate loop_rate(50);
      ros::spinOnce();
      loop_rate.sleep();

      ros::Duration period = ros::Time::now() - begin;
      double period_secs = period.toSec();
      if (period_secs > 1) {
        break;
      }
    }

    REQUIRE(tester.left_vel == 200); 
    REQUIRE(tester.right_vel == 200);
  }

  SECTION( "pure rotation" ) {
    ros::Time begin = ros::Time::now();
    while (ros::ok()) {
      tester.pub_cmd_vel(0, 1);

      ros::Rate loop_rate(50);
      ros::spinOnce();
      loop_rate.sleep();

      ros::Duration period = ros::Time::now() - begin;
      double period_secs = period.toSec();
      if (period_secs > 1) {
        break;
      }
    }

    REQUIRE(tester.left_vel == -200); 
    REQUIRE(tester.right_vel == 200);
  }

}

TEST_CASE( "Test encoders", "[sensor_data, joint_states]" ) {
  ros::NodeHandle n;
  Handler tester(n);

  ros::Time begin = ros::Time::now();
  int left_idx = 0;
  int right_idx = 0;
  while (ros::ok()) {
    tester.pub_sensor_data(left_idx, right_idx);
    left_idx += 1;
    right_idx += 1;

    ros::Rate loop_rate(50);
    ros::spinOnce();
    loop_rate.sleep();

    ros::Duration period = ros::Time::now() - begin;
    double period_secs = period.toSec();
    if (period_secs > 1) {
      break;
    }
  }

  REQUIRE(tester.left_vel_js == Approx(0.07).epsilon(0.05)); 
  REQUIRE(tester.right_vel_js == Approx(0.07).epsilon(0.05)); 

}

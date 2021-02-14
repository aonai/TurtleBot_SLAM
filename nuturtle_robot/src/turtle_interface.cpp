/**
 * \brief This node is going to make a turtlebot move based on wheel velocities calculated from input twist.
 * 
 * PUBLISHERS:
 * + joint_states (sensor_msgs::JointState) ~ joint states of robot 
 * + wheel_cmd (nuturtlebot::WheelCommands) ~ left and right wheel velocities of turtle
 * 
 * SUBSRIBERS:
 * + cmd_vel (geometry_msgs::Twist) ~ velocity of turtle in form of twist
 * + sensor_data (nuturtlebot::SensorData) ~ sensor data of turtle
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


/// \brief Helper class for node turtle_interface
class Handler {
    public:
        explicit Handler(ros::NodeHandle & n);

    private:
        //variables 
        double wheel_base = 0.08;
        double wheel_radius = 0.033;
        double max_encoder = 2.84;     
        double gear_ratio = 258.5;
        int rpm = 57; 
        int max_step = 4096;
        double left_encoder_pre = 0;
        double right_encoder_pre = 0;
        double left_vel = 0;
        double right_vel = 0;
        double left_pos = 0;
        double right_pos = 0;
        bool pub_wheel = true;
        ros::Time begin = ros::Time::now();
        std::string left_wheel_joint = "wheel_left_link";
        std::string right_wheel_joint = "wheel_right_link";
        rigid2d::DiffDriveVel wheel_vel {0, 0};
        rigid2d::DiffDrive turtle;           
        // ROS members
        ros::Subscriber cmd_vel_sub;
        ros::Subscriber sensor_data_sub;
        ros::Publisher wheel_cmd_pub;
        ros::Publisher joint_state_pub;
        // helper functions
        void cmd_vel_sub_callback(const geometry_msgs::Twist & vel);
        void sensor_data_sub_callback(const nuturtlebot::SensorData  & msg);
        void pub_wheel_cmd();
        void pub_joint_state();

};

/// \brief Init Handler class
/// \param n - turtle_interface NodeHandle
Handler::Handler(ros::NodeHandle & n) : turtle(wheel_base, wheel_radius) {
    cmd_vel_sub = n.subscribe("cmd_vel", 10, &Handler::cmd_vel_sub_callback, this);
    sensor_data_sub = n.subscribe("sensor_data", 10, &Handler::sensor_data_sub_callback, this);
    wheel_cmd_pub = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 10);
    joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

    while (ros::ok()) {
        ros::param::get("~pub_wheel", pub_wheel);
        if (pub_wheel) {        
            pub_wheel_cmd();    // publish wheel_cmd message only if follow_circle node is not running
        }
        pub_joint_state();

        ros::Rate loop_rate(50);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}

/// \brief Helper function for publishing wheel commands
void Handler::pub_wheel_cmd() {
    nuturtlebot::WheelCommands msg;
    msg.left_velocity = wheel_vel.vL*gear_ratio/(2*rigid2d::PI*rpm/60);
    msg.right_velocity = wheel_vel.vR*gear_ratio/(2*rigid2d::PI*rpm/60);
    wheel_cmd_pub.publish(msg);
    // ROS_INFO_STREAM("pub wheel cmd: " << msg.left_velocity <<" "<< msg.right_velocity << " from " << wheel_vel.vL << " " << wheel_vel.vR);
}


/// \brief Callback function for cmd_vel subscriber
/// Calculates left and right velocities of wheels for turtle to reach cmd_vel.
/// Wheel velocities should not exceed maximum rotational velocity of turtle.
/// \param vel - velocity of turtle
void Handler::cmd_vel_sub_callback(const geometry_msgs::Twist  & vel) {
    double omg = vel.angular.z;
    double vx = vel.linear.x;

    rigid2d::Twist2D t {omg, vx, 0};
    wheel_vel = turtle.vel_from_twist(t);

    if (wheel_vel.vL >= max_encoder) {
        wheel_vel.vL = max_encoder;
    }
    else if (wheel_vel.vL <= -max_encoder) {
        wheel_vel.vL = -max_encoder;
    }
    if (wheel_vel.vR >= max_encoder) {
        wheel_vel.vR = max_encoder;
    }
    else if (wheel_vel.vR <= -max_encoder) {
        wheel_vel.vR = -max_encoder;
    }

    // ROS_INFO_STREAM("calc vel: " << wheel_vel.vL <<" "<< wheel_vel.vR);
}

/// \brief Callback function for sensor_data subscriber
/// Calculates left and right velocities from encoders
/// \param msg - sensor data of turtle
void Handler::sensor_data_sub_callback(const nuturtlebot::SensorData  & msg) {
    double left_encoder = msg.left_encoder-left_encoder_pre;
    double right_encoder = msg.right_encoder-right_encoder_pre;

    ros::Duration period = ros::Time::now() - begin;
    double period_secs = period.toSec();
    if (period_secs > 1e-3) {
        begin = ros::Time::now();
        left_vel = (left_encoder/max_step)*(2*rigid2d::PI*rpm/60)/period_secs;
        right_vel = (right_encoder/max_step)*(2*rigid2d::PI*rpm/60)/period_secs;
        left_pos += left_vel*period_secs;
        right_pos += right_vel*period_secs;

        left_encoder_pre = msg.left_encoder;
        right_encoder_pre = msg.right_encoder;

        // ROS_INFO_STREAM("calc encoder " << left_encoder << " " <<right_encoder<<" to "<<left_vel<<" "<<right_vel << " period = " << period_secs);
    }
}

/// \brief Helper function for publishing joint states
void Handler::pub_joint_state() {
  sensor_msgs::JointState js;
  std::vector<std::string> name = {left_wheel_joint, right_wheel_joint};
  std::vector<double> pos = {left_pos, right_pos};
  std::vector<double> vel = {left_vel, right_vel};
  js.name = name;
  js.position = pos;
  js.velocity = vel;
  joint_state_pub.publish(js);
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle_interface");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
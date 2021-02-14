/**
 * \brief This node is make a turtlebot move in a circle with input radius.
 * 
 * PUBLISHERS:
 * + wheel_cmd (nuturtlebot::WheelCommands) ~ left and right wheel velocities of turtle
 * 
 * SERVICES:
 * + control (nuturtle_robot::Control) ~ make turtlebot move clockwise or counterclowise, or make it stop
 * 
**/


#include <vector>
#include <string>
#include "ros/ros.h"
#include "rigid2d/diff_drive.hpp"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtle_robot/ControlMsg.h"
#include "nuturtle_robot/Control.h"


/// \brief Helper class for node follow_circle
class Handler {
    public:
        explicit Handler(ros::NodeHandle & n);

    private:
        //variables 
        double wheel_base = 0.08;
        double wheel_radius = 0.033;
        int max_encoder = 250;  
        double gear_ratio = 258.5;
        int rpm = 57; 
        double circle_radius = 0.3;
        rigid2d::DiffDriveVel wheel_vel {0, 0};
        rigid2d::DiffDrive turtle;           
        // ROS members
        ros::Publisher wheel_cmd_pub;
        ros::ServiceServer control_service;
        // helper functions
        void pub_wheel_cmd();
        bool control_service_callback(nuturtle_robot::Control::Request  & req, nuturtle_robot::Control::Response & res);

};

/// \brief Init Handler class
/// \param n - follow_circle NodeHandle
Handler::Handler(ros::NodeHandle & n) : turtle(wheel_base, wheel_radius) {
    wheel_cmd_pub = n.advertise<nuturtlebot::WheelCommands>("wheel_cmd", 10);
    control_service = n.advertiseService("control", &Handler::control_service_callback, this);
    
    while (ros::ok()) {
        ros::param::get("~circle_radius", circle_radius);
        pub_wheel_cmd();

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
    // ROS_INFO_STREAM("circle encoder: " << msg.left_velocity <<" "<< msg.right_velocity << " from " << wheel_vel.vL <<" "<<wheel_vel.vR);
}


/// \brief Callback function for control servive
/// Calculates left and right velocities of wheels for turtle to drive in a circle.
/// Wheel velocities should not exceed maximum rotational velocity of turtle.
/// The angular velocity of turtle is set to 0.2. 
/// \param req - request of control service 
/// \param res - responce of control service 
bool Handler::control_service_callback(nuturtle_robot::Control::Request  & req, nuturtle_robot::Control::Response & res) {
    if (req.msg.stop) {
        wheel_vel.vL = 0;
        wheel_vel.vR = 0;
    }
    else {
        double omg = -0.2;
        if (req.msg.clockwise) {
            omg = 0.2;
        }
        double vx = omg*circle_radius;

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

        // ROS_INFO_STREAM("circle vel: " << wheel_vel.vL <<" "<< wheel_vel.vR);
    }
    return true;
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "follow_circle");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
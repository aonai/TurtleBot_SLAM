/**
 * \brief This node is make a turtlebot move in a circle with input radius.
 * 
 * PUBLISHERS:
 * + cmd_vel (geometry_msgs::Twist) ~ velocity of turtle in form of twist
 * 
 * SERVICES:
 * + control (nuturtle_robot::Control) ~ make turtlebot move clockwise or counterclowise, or make it stop
 * 
**/


#include <vector>
#include <string>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nuturtle_robot/ControlMsg.h"
#include "nuturtle_robot/Control.h"


/// \brief Helper class for node follow_circle
class Handler {
    public:
        explicit Handler(ros::NodeHandle & n);

    private:
        //variables 
        double vx = 0;
        double omg = 0;
        double circle_radius = 0.3;
        // ROS members
        ros::Publisher cmd_vel_pub;
        ros::ServiceServer control_service;
        // helper functions
        void pub_cmd_vel();
        bool control_service_callback(nuturtle_robot::Control::Request  & req, nuturtle_robot::Control::Response & res);

};

/// \brief Init Handler class
/// \param n - follow_circle NodeHandle
Handler::Handler(ros::NodeHandle & n) {
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    control_service = n.advertiseService("control", &Handler::control_service_callback, this);
    
    while (ros::ok()) {
        ros::param::get("~circle_radius", circle_radius);
        pub_cmd_vel();

        ros::Rate loop_rate(50);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}

/// \brief Helper function for publishing cmd_vel
void Handler::pub_cmd_vel() {
    geometry_msgs::Twist msg;
    msg.linear.x = vx;
    msg.angular.z = omg;

    cmd_vel_pub.publish(msg);
    // ROS_INFO_STREAM("pub vel: " << vx <<" "<< omg);
}


/// \brief Callback function for control servive
/// Calculates velocities of turtle to drive in a circle.
/// \param req - request of control service 
/// \param res - responce of control service 
bool Handler::control_service_callback(nuturtle_robot::Control::Request  & req, nuturtle_robot::Control::Response & res) {
    if (req.msg.stop) {
        vx = 0;
        omg = 0;
    }
    else {
        omg = -0.2;
        if (req.msg.clockwise) {
            omg = 0.2;
        }
        vx = omg*circle_radius;

    }
    return true;
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "follow_circle");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
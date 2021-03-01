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



#include "ros/ros.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuslam/kalman.hpp"


/// \brief Helper class for node tube_world
class Handler {
  public:
    explicit Handler(ros::NodeHandle & n);

  private:
    // variables       
    // ROS members   
    // helper functions
};

/// \brief Init Handler class
/// \param n - tube_world NodeHandle
Handler::Handler(ros::NodeHandle & n) {
    ROS_INFO("hello");
    kalman::test();
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}

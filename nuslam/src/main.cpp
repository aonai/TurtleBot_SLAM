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
    rigid2d::DiffDrive fake {0.08, 0.033};
    rigid2d::Twist2D t {0, 1, 0}; //omg=0, vx=1
    rigid2d::DiffDriveVel wheel_vel = fake.vel_from_twist(t);
    fake.update(wheel_vel.vL, wheel_vel.vR);
    
    rigid2d::Transform2D config = fake.config();
    ROS_INFO_STREAM("updated fake: " << config.theta() << " " << config.x() << " " << config.y());

    arma::vec map_state {4.0, 4.0};
    kalman::StateVec s {map_state, 0.01, 0.01};


    s.ekf_update(t, 5, 0.9273);

}



int main(int argc, char **argv) {
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}

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
#include <cmath>
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
    void align_map(arma::vec robot_state, arma::vec map_state);
};

/// \brief Init Handler class
/// \param n - tube_world NodeHandle
Handler::Handler(ros::NodeHandle & n) {
    rigid2d::DiffDrive fake {0.08, 0.033};
    rigid2d::Twist2D t {0, 1, 0}; 
    rigid2d::DiffDriveVel wheel_vel = fake.vel_from_twist(t);
    fake.update(wheel_vel.vL, wheel_vel.vR);
    
    rigid2d::Transform2D config = fake.config();
    ROS_INFO_STREAM("updated fake: " << config.theta() << " " << config.x() << " " << config.y());

    arma::vec map_state {4.0, 4.0};
    kalman::StateVec s {map_state, 0.01, 0.01};

    arma::vec odom {config.theta(), config.x(), config.y()};
    arma::vec d {5.66, sqrt(2)};
    arma::vec ang {0.78, -0.78};
    s.ekf_update(t, d, ang);
    align_map(s.get_robot_state(), s.get_map_state());

}

void Handler::align_map(arma::vec robot_state, arma::vec map_state) {
  ROS_INFO_STREAM("robot " << robot_state);
  ROS_INFO_STREAM("map " << map_state);

  arma::vec land {4.0, 4.0, 1, -1};
  arma::vec odom {0, 0, 0};

  int n_obst = map_state.size()/2;
  arma::vec remap_state (3, arma::fill::zeros);
  for (int i = 0; i < n_obst; i++) {
    double ms_ang = atan2(map_state[2*i+1]-robot_state[2],map_state[2*i]-robot_state[1]) - robot_state[0];
    double land_ang = atan2(land[2*i+1]-odom[2],land[2*i]-odom[1]) - odom[0];
    remap_state(0) = remap_state(0) + ms_ang - land_ang;
    remap_state(1) = remap_state(1) + map_state(2*i) - land(2*i);
    remap_state(2) = remap_state(2) + map_state(2*i+1) - land(2*i+1);
    // ROS_INFO_STREAM(ms_ang << " " << land_ang <<" remap " << remap_state);
  }
  remap_state = -remap_state/n_obst;
  ROS_INFO_STREAM("remap " << remap_state);
  arma::vec tmp (3);
  tmp(0) = robot_state(0) + remap_state(0);
  tmp(1) = robot_state(1) + remap_state(1);
  tmp(2) = robot_state(2) + remap_state(2);
  ROS_INFO_STREAM("remaped robot " << tmp);

}



int main(int argc, char **argv) {
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}

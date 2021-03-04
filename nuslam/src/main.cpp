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
    rigid2d::Twist2D t {0, 0.5, 0}; 
    rigid2d::DiffDriveVel wheel_vel = fake.vel_from_twist(t);
    fake.update(wheel_vel.vL, wheel_vel.vR);
    
    rigid2d::Transform2D config = fake.config();
    // ROS_INFO_STREAM("updated fake: " << config.theta() << " " << config.x() << " " << config.y());

    arma::vec map_state {1, 1, 3, 3, 2, 2};
    kalman::StateVec s {map_state, 0, 0};

    arma::vec d {0.5, 2, 4};
    arma::vec ang {0.788, 0.123, -0.9};
    s.ekf_update(t, d, ang);
    
    arma::vec check =  s.measurement_vec(0);
    arma::vec check1 =  s.measurement_vec(1);
    arma::vec check2 =  s.measurement_vec(2);
    std::cout << 2*rigid2d::PI+check(1) << " or vec ="  <<check.t() <<std::endl;
    std::cout << 2*rigid2d::PI+check1(1) << " or vec ="  <<check1.t() <<std::endl;
    std::cout << 2*rigid2d::PI+check2(1) << " or vec ="  <<check2.t() <<std::endl;

    rigid2d::Vector2D v {1, 1};
    rigid2d::Transform2D trans {rigid2d::PI/4};
    std::cout << trans(v) << std::endl;

    // arma::vec m = s.get_map_state();
    // rigid2d::Vector2D robot_v {m(1), m(2)};
    // rigid2d::Transform2D T_map_obst {robot_v, map_state(0)};
    // std::cout << "T_map_obst " << T_map_obst << std::endl;
    // rigid2d::Vector2D o {1*cos(3), 1*sin(3)};
    // rigid2d::Transform2D To {o, 3};
    // std::cout << "To " << To << std::endl;
    // rigid2d::Transform2D T_map_odom = T_map_obst * To.inv();
    // std::cout << "T_map_odom " << T_map_odom << std::endl;

    // double a = 0;
    // for (unsigned i = 0; i < 10; i++) {
    //   a += 1;
    //   a = rigid2d::normalize_angle(a);

    //   arma::vec d {1, -1, -1};
    //   arma::vec ang {a, -1, -1};
      
    //   rigid2d::Twist2D t_tmp {1, 0, 0};
    //   s.ekf_update(t_tmp, d, ang);
    //   // s.reset_cov();
    //   arma::vec check =  s.measurement_vec(0);
    //   // check(1) = rigid2d::normalize_angle(check(1));
    //   std::cout << a << "-------- update ang=" << 2*rigid2d::PI+check(1) << " or vec="  <<check.t() <<std::endl;
    // }
    // // std::cout << "test " << s.get_map_state();

}



int main(int argc, char **argv) {
  ros::init(argc, argv, "main");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}

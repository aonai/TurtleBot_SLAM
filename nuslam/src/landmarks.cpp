

#include <string>
#include <cmath>
#include <armadillo>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

/// \brief Helper class for node landmarks
class Handler {
  public:
      explicit Handler(ros::NodeHandle & n);

  private:
    //variables  
    double wheel_base = 0.16;
    double wheel_radius = 0.033;
    double laser_range_min = 0.12;
    double laser_range_max = 3.5;
    double laser_angle_increment = 0.01744;
    int laser_samples_num = 360;
    double rad_left_thresh = 0.0;
    double rad_right_thresh = 0.0;
    double rad_left_old = 0.0;
    double rad_right_old = 0.0;
    arma::vec laser_data;
    double laser_cluster_threshold = 0.05;
    rigid2d::DiffDrive odom;
    // ROS members
    ros::Subscriber joint_states_sub;
    ros::Subscriber laser_scan_sub;
    //helper functions
    void find_param(ros::NodeHandle & n);
    void joint_states_sub_callback(const sensor_msgs::JointState & msg);
    void laser_scan_sub_callback(const sensor_msgs::LaserScan & data);
    void group_clusters();
};

/// \brief Init Handler class
/// \param n - landmarks NodeHandle
Handler::Handler(ros::NodeHandle & n) : odom(wheel_base/2, wheel_radius) {
  joint_states_sub = n.subscribe("joint_states", 10, &Handler::joint_states_sub_callback, this);
  laser_scan_sub = n.subscribe("laser_scan", 10, &Handler::laser_scan_sub_callback, this);


  while (ros::ok()) {
    find_param(n);

    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
  }
    
}

/// \brief Find parameters 
/// \param n - tube_world NodeHandle
void Handler::find_param(ros::NodeHandle & n) {
  n.param("wheel_base", wheel_base, 0.16);
  n.param("wheel_radius", wheel_radius, 0.033);
  n.param("laser_range_min", laser_range_min, 0.12);
  n.param("laser_range_max", laser_range_max, 3.5);
  n.param("laser_angle_increment", laser_angle_increment, 0.01744);
  n.param("laser_samples_num", laser_samples_num, 360);
}

void Handler::laser_scan_sub_callback(const sensor_msgs::LaserScan & data) {
    laser_data = arma::vec (laser_samples_num);

    for (unsigned i = 0; i < laser_samples_num; i++) {
        laser_data(i) = data.ranges[i];
    }

    group_clusters();
}

/// \brief Callback function for joint_states subscriber.
/// Updates robot config given updated wheel angles.
/// Also updates robot config with noise and collision for checking correctness of Kalman slam.
/// After calcualting twist from joint states, input twist and fake measurement into
/// Kalman Filter to calcualte slam turlte pose and slam obstacle locations.
/// \param msg - message from subscriber
void Handler::joint_states_sub_callback(const sensor_msgs::JointState & msg) {
  double rad_left = msg.position[0] - rad_left_thresh;
  double rad_right = msg.position[1] - rad_right_thresh;

  if (rad_left != 0 && rad_right != 0) {
    // odom turtle 
    double rad_left_diff = rad_left - rad_left_old;
    rad_left_diff = rigid2d::normalize_angle(rad_left_diff);
    double rad_right_diff = rad_right - rad_right_old;
    rad_right_diff = rigid2d::normalize_angle(rad_right_diff);
    odom.update(rad_left_diff, rad_right_diff);
    rad_left_old = rad_left;
    rad_right_old = rad_right;

    // rigid2d::Transform2D config = odom.config();
    // ROS_INFO_STREAM("odom = " << config);
  }
}

void Handler::group_clusters() {
    ROS_INFO_STREAM("check laser data" << laser_data.t());

}


int main(int argc, char **argv) {
  ros::init(argc, argv, "landmarks");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
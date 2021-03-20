/**
 * \brief This node is going to find out landmark locations in world from laser scan messages. 
 * 
 * 
 * PUBLISHERS:
 * + circle_obst (visualization_msgs::MarkerArray) ~ landmark locations calculated from laser scan messages 
 * + lm_measurement (std_msgs::Float64MultiArray) ~ distance and angle from robot to landmarks 
 * 
 * SUBSRIBERS:
 * + joint_states (sensor_msgs::JointState) ~ joint states of turtle with and without noise
 * + "laser_scan" or "scan" (sensor_msgs::LaserScan) ~ laser scan messages with respect to turtle
 *  
 * 
 * PARAMETERS:
 * + wheel_base (string) ~ distance between wheels of robot  
 * + wheel_radius (string) ~ radius of wheels  
 * + obst_radius (double) ~ radius of tube obstacle   
 * + laser_range_min (double) ~ minimum range of laser scan
 * + laser_range_max (double) ~ maximum range of laser scan
 * + laser_angle_increment (double) ~ angle increment between each measurement in rad
 * + laser_samples_num (int) ~ number of measurements
 * + wall_size (double) ~ dimension of a wall 
 * + is_simu (bool) ~ whether laser scan messages are generated from simulation or real world
 * 
 * 
**/

#include <string>
#include <cmath>
#include <armadillo>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/LaserScan.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "nuslam/circle.hpp"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float64MultiArray.h"

/// \brief Helper class for node landmarks
class Handler {
  public:
      explicit Handler(ros::NodeHandle & n);

  private:
    //variables  
    double wheel_base = 0.16;
    double wheel_radius = 0.033;
    double obst_radius = 0.0762;
    double laser_range_min = 0.12;
    double laser_range_max = 3.5;
    double laser_angle_increment = 0.01744;
    int laser_samples_num = 360;
    double wall_size = 2.5;
    bool is_simu = true;
    double rad_left_thresh = 0.0;
    double rad_right_thresh = 0.0;
    double rad_left_old = 0.0;
    double rad_right_old = 0.0;
    bool init = true;
    arma::vec laser_data;
    double laser_cluster_threshold = 0.05;
    rigid2d::DiffDrive odom;
    arma::vec cluster_is_circle;
    arma::vec cluster_center;
    arma::vec cluster_radius;
    arma::vec world_center;
    // ROS members
    ros::Subscriber joint_states_sub;
    ros::Subscriber laser_scan_sub;
    ros::Publisher circle_obst_pub;
    ros::Publisher lm_measurenmt_pub;
    //helper functions
    void find_param(ros::NodeHandle & n);
    void joint_states_sub_callback(const sensor_msgs::JointState & msg);
    void laser_scan_sub_callback(const sensor_msgs::LaserScan & data);
    void pub_circle_marker();
    
};

/// \brief Init Handler class
/// \param n - landmarks NodeHandle
Handler::Handler(ros::NodeHandle & n) : odom(wheel_base/2, wheel_radius) {
  joint_states_sub = n.subscribe("joint_states", 10, &Handler::joint_states_sub_callback, this);
  circle_obst_pub = n.advertise<visualization_msgs::MarkerArray>( "circle_obst", 10 );
  lm_measurenmt_pub =  n.advertise<std_msgs::Float64MultiArray>( "lm_measurement", 10 );

  while (ros::ok()) {
    find_param(n);
    pub_circle_marker();
    if (is_simu && init) {
      laser_scan_sub = n.subscribe("laser_scan", 10, &Handler::laser_scan_sub_callback, this);
      init = false;
    }
    else if (is_simu == false && init) {
      laser_scan_sub = n.subscribe("scan", 10, &Handler::laser_scan_sub_callback, this);
      init = false;
    }

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
  n.param("obst_radius", obst_radius, 0.0762);
  n.param("laser_range_min", laser_range_min, 0.12);
  n.param("laser_range_max", laser_range_max, 3.5);
  n.param("laser_angle_increment", laser_angle_increment, 0.01744);
  n.param("laser_samples_num", laser_samples_num, 360);
  n.param("wall_size", wall_size, 2.5);
  n.param("is_simu", is_simu, true);
}

/// \brief Calcualte landmark locations from a laser scan message
/// First it will group measurements into clusters and check whether a cluster is a circle.
/// Then, calculate and publish distance and angle from turtle to landmarks. 
/// \param data - laser scan message
void Handler::laser_scan_sub_callback(const sensor_msgs::LaserScan & data) {
    laser_data = arma::vec (laser_samples_num);

    for (unsigned i = 0; i < laser_samples_num; i++) {
        laser_data(i) = data.ranges[i];
    }

    // group clusters
    arma::vec angles(laser_data.size());
    for (unsigned i = 0; i < angles.size(); i ++) {
      angles(i) = i * laser_angle_increment;
    }
    arma::field <arma::vec> cluster_groups = circle::group_clusters(laser_data, angles);
    cluster_is_circle = arma::vec(cluster_groups.size());
    cluster_center = arma::vec(cluster_groups.size()*2);
    cluster_radius = arma::vec(cluster_groups.size());
    world_center = arma::vec(cluster_groups.size()*2);
    std_msgs::Float64MultiArray float_array;

    for (unsigned i = 0; i < cluster_groups.size(); i++) {
      
      circle::Cluster group {cluster_groups(i)};
      group.fit_circle();
      cluster_is_circle(i) = group.check_is_circle();
      cluster_center(2*i) = group.get_center_x();
      cluster_center(2*i+1) = group.get_center_y();
      cluster_radius(i) = group.get_circle_radius();
      
      // convert to world frame
      rigid2d::Transform2D config = odom.config();
      double x_to_turtle = cluster_center(2*i);
      double y_to_turtle = cluster_center(2*i+1);
      double ang_to_turtle = atan2(x_to_turtle, y_to_turtle) - config.theta();
      double r_to_turtle = sqrt(pow(x_to_turtle,2)+pow(y_to_turtle,2));

      double center_x = config.x() + r_to_turtle*sin(ang_to_turtle); 
      double center_y = config.y() + r_to_turtle*cos(ang_to_turtle); 

      world_center(2*i) = center_x;
      world_center(2*i+1) = center_y;


      // filter out circle that are not arc
      group.classify_arc();
      cluster_is_circle(i) = group.check_is_circle();
     
      // filter out circle too close to wall or out of range
      if ( fabs(center_x) >= wall_size*0.9 || fabs(center_y) >= wall_size*0.9) {
        cluster_is_circle(i) = false;
      }

      // check circle radius 
      if (cluster_radius(i) > obst_radius*1.5 || cluster_radius(i) < obst_radius*0.5) {
        // std::cout << "change is circle at " << i <<ssss " R = " << cluster_radius(i) << std::endl;
        cluster_is_circle(i) = false;
      }

      // filter out circles that are out of range
      if ( sqrt(pow(x_to_turtle,2)+pow(y_to_turtle,2)) >= laser_range_max 
          || sqrt(pow(x_to_turtle,2)+pow(y_to_turtle,2)) <= laser_range_min ) {
        cluster_is_circle(i) = false;
      }

      if (cluster_is_circle(i)) {
        // std::cout << "--- center = " << world_center(2*i) << " " << world_center(2*i+1) ;
        // std::cout << "  circle_r = " << cluster_radius(i) << std::endl;

        // lm_measurement info
        double x_dist = center_x - config.x();
        double y_dist = center_y - config.y();
        double measure_dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));
        double angle = atan2(y_dist, x_dist) - config.theta();
        angle = rigid2d::normalize_angle(angle);
        double dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));

        float_array.data.push_back(measure_dist);
        float_array.data.push_back(angle);
      }

    }
    lm_measurenmt_pub.publish(float_array);


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

/// \brief Helper function for publishing circle marker.
void Handler::pub_circle_marker() {
  visualization_msgs::MarkerArray marker_array;
  for (unsigned i = 0; i < cluster_is_circle.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "circle_obst";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    if (cluster_is_circle(i) == true) {
      marker.action = visualization_msgs::Marker::ADD;
    }
    else {
      marker.action = visualization_msgs::Marker::DELETE;
    }
    marker.pose.position.x = world_center(2*i);
    marker.pose.position.y = world_center(2*i+1);
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = obst_radius*2;
    marker.scale.y = obst_radius*2;
    marker.scale.z = 1.5;
    marker.color.a = 0.5; 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker_array.markers.push_back(marker);
  }
  circle_obst_pub.publish( marker_array );
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "landmarks");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
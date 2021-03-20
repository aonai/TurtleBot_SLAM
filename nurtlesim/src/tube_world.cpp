/**
 * \brief This node is going to generate a fake turtle that is controlled by /cmd_vel and 
 * publishes joint states and fake sensor information containing relative distance and angle
 * to obstacles. These information have Gaussian noise in commanded twist, uniform random noise in slipping,
 * and Gaussian noise in fake sensor. 
 * 
 * PUBLISHERS:
 * + joint_states (sensor_msgs::JointState) ~ joint states of turtle with and without noise
 * + obstacles (visualization_msgs::MarkerArray) ~ ground truth locations of obstacles and robot
 * + real_path (nav_msgs::Path) ~ real pose of turtle with noise
 * + fake_sensor (visualization_msgs::MarkerArray) ~ fake sensor information with noise 
 * + fake_measurement (std_msgs::Float64MultiArray) ~ measured distance and angle from robot to observed obstacles 
 * 
 * SUBSRIBERS:
 * + cmd_vel (geometry_msgs::Twist) ~ the velocity of turtle
 *  
 * 
 * PARAMETERS:
 * + wheel_base (string) ~ distance between wheels of robot  
 * + wheel_radius (string) ~ radius of wheels  
 * + left_wheel_joint (string) ~ name of robot left wheel joint  
 * + right_wheel_joint (string) ~ name of robot right wheel joint  
 * + left_wheel_joint_slip (string) ~ name of robot left wheel joint with noises  
 * + right_wheel_joint_slip (string) ~ name of robot right wheel joint with noises  
 * + noise_mean (double) ~ mean of Gaussian noise for commanded twist   
 * + noise_variance (double) ~ variance of Gaussian noise for commanded twist  
 * + slip_min (double) ~ minimum uniform radom noise ratio for slipping  
 * + slip_max (double) ~ maximum uniform radom noise ratio for slipping   
 * + x_coords (double list) ~ x locations of obstacles in world  
 * + y_coords (double list) ~ y locations of obstacles in world  
 * + covariance_matrix_0 (double list) ~ first row of covariance matrix of obstacle lcoations   
 * + covariance_matrix_1 (double list)~ second row of covariance matrix of obstacle lcoations  
 * + obst_radius (double) ~ radius of tube obstacle   
 * + obst_max_dist (double) ~ radius of observation area for robot; obstacles within this range will be used to publish sensor information   
 * + tube_var (double) ~ variance of Gaussian noise for obstacles   
 * 
 * 
**/


#include <vector>
#include <string>
#include <random>
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float64MultiArray.h"
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include "sensor_msgs/LaserScan.h"
#include <armadillo>


/// \brief Helper class for node tube_world
class Handler {
  public:
    explicit Handler(ros::NodeHandle & n);

  private:
    // variables 
    double wheel_base = 0.16;
    double wheel_radius = 0.033;
    double robot_radius = 0.12;
    std::string left_wheel_joint;
    std::string left_wheel_joint_slip;
    std::string right_wheel_joint;
    std::string right_wheel_joint_slip;
    double current_rad_left = 0;
    double current_rad_left_slip = 0;
    double current_rad_right = 0;   
    double current_rad_right_slip = 0;   
    double noise_mean = 0.0;
    double noise_variance = 0.01;
    double slip_min = -0.01;
    double slip_max = 0.01;
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::vector<std::vector<double>> covariance_matrix = {{1.0, 1.0}, {1.0, 1.0}};;
    double obst_radius = 0.0762;
    double obst_max_dist = 3.0;
    double tube_var = 0.001;
    bool use_odom = false;
    rigid2d::DiffDrive fake;             
    rigid2d::DiffDrive fake_slip;    
    double laser_range_min = 0.12;
    double laser_range_max = 3.5;
    double laser_angle_increment = 0.01744;
    int laser_samples_num = 360;
    double laser_resolution = 0.015;
    double laser_noise_level = 0.0;
    double wall_size = 2.5;  
    arma::vec x_arr;
    arma::vec y_arr;
    // ROS members     
    ros::Time begin = ros::Time::now();
    ros::Subscriber cmd_vel_sub;
    ros::Publisher joint_state_pub;
    ros::Publisher obst_pub;
    ros::Publisher path_pub;
    nav_msgs::Path path_msg; 
    tf2_ros::TransformBroadcaster br;
    ros::Publisher fake_sensor_pub;
    ros::Publisher fake_measurenmt_pub;
    ros::Publisher laser_scan_pub;
    ros::Timer timer;
    // helper functions
    void find_param(ros::NodeHandle & n);
    void cmd_vel_sub_callback(const geometry_msgs::Twist & vel);
    void pub_joint_state();
    void pub_obst_marker();
    void pub_path();
    void tf_broadcast();
    void pub_fake_sensor();
    void check_collision(double period_secs);
    void update_turtle(double period_secs, double omg, double vx, rigid2d::DiffDrive & turtle, bool add_noise);
    void timer_callback(const ros::TimerEvent& event);
    std::mt19937 & get_random();
};

/// \brief Init Handler class
/// \param n - tube_world NodeHandle
Handler::Handler(ros::NodeHandle & n) : fake(wheel_base/2, wheel_radius), fake_slip(wheel_base/2, wheel_radius) {
  cmd_vel_sub = n.subscribe("cmd_vel", 10, &Handler::cmd_vel_sub_callback, this);
  joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
  obst_pub = n.advertise<visualization_msgs::MarkerArray>( "obstacles", 10 );
  path_pub = n.advertise<nav_msgs::Path>("real_path", 10);
  path_msg.header.stamp=ros::Time::now();
  path_msg.header.frame_id="world"; 
  fake_sensor_pub = n.advertise<visualization_msgs::MarkerArray>( "fake_sensor", 10 );
  fake_measurenmt_pub =  n.advertise<std_msgs::Float64MultiArray>( "fake_measurement", 10 );
  laser_scan_pub = n.advertise<sensor_msgs::LaserScan>( "laser_scan", 10 );
  timer =  n.createTimer(ros::Duration(0.2), &Handler::timer_callback, this);

  while (ros::ok()) {
    find_param(n);

    pub_fake_sensor();
    pub_joint_state();
    pub_obst_marker();
    pub_path();
    tf_broadcast();


    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
  }
    
}

/// \brief Helper function for generating random gaussian noise
/// \return A random generator 
std::mt19937 & Handler::get_random()
 {
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     return mt;
 }

/// \brief Find parameters 
/// \param n - tube_world NodeHandle
void Handler::find_param(ros::NodeHandle & n) {
  std::string default_left = "wheel_left_link";
  std::string default_right = "wheel_right_link";
  std::string default_left_slip = "wheel_left_link_slip";
  std::string default_right_slip = "wheel_right_link_slip";
  std::vector<double> default_x_coords = {0.0, 1.0, 2.0};
  std::vector<double> default_y_coords = {0.0, 1.0, 2.0};
  std::vector<std::vector<double>> default_cm = {{1.0, 1.0}, {1.0, 1.0}};
  n.param("wheel_base", wheel_base, 0.16);
  n.param("wheel_radius", wheel_radius, 0.033);
  n.param("left_wheel_joint", left_wheel_joint, default_left);
  n.param("right_wheel_joint", right_wheel_joint, default_right);
  n.param("left_wheel_joint_slip", left_wheel_joint_slip, default_left_slip);
  n.param("right_wheel_joint_slip", right_wheel_joint_slip, default_right_slip);
  n.param("noise_mean", noise_mean, 0.0);
  n.param("noise_variance", noise_variance, 0.01);
  n.param("slip_min", slip_min, -0.05);
  n.param("slip_max", slip_max, 0.05);
  n.param("x_coords", x_coords, default_x_coords);
  n.param("y_coords", y_coords, default_y_coords);
  n.param("covariance_matrix_0", covariance_matrix[0], default_cm[0]);
  n.param("covariance_matrix_1", covariance_matrix[1], default_cm[1]);
  n.param("obst_radius", obst_radius, 0.0762);
  n.param("obst_max_dist", obst_max_dist, 2.0);
  n.param("tube_var", tube_var, 0.01);
  n.param("use_odom", use_odom, false);
  n.param("laser_range_min", laser_range_min, 0.12);
  n.param("laser_range_max", laser_range_max, 3.5);
  n.param("laser_angle_increment", laser_angle_increment, 0.01744);
  n.param("laser_samples_num", laser_samples_num, 360);
  n.param("laser_resolution", laser_resolution, 0.015);
  n.param("laser_noise_level", laser_noise_level, 0.01);
  n.param("wall_size", wall_size, 2.5);
}

/// \brief Callback function for cmd_vel subscriber
/// Compute wheel velocities from given twist (cmd_vel), then update turtle (fake) config
/// and wheel angles and update another turtle (fake_slip) config and wheel angles with noise. 
/// Check whether turtle is colliding with an obstacles. If yes, turtle should move along a tangent
/// line between itself (simulated as a sphere) and the obstacles. 
/// \param vel - velocity of turtle
void Handler::cmd_vel_sub_callback(const geometry_msgs::Twist  & vel) {
  ros::Duration period = ros::Time::now() - begin;
  double period_secs = period.toSec();
   if (period_secs > 1e-3) {
      begin = ros::Time::now();
      double omg = vel.angular.z;
      double vx = vel.linear.x;
      update_turtle(period_secs, omg, vx, fake, false);
      update_turtle(period_secs, omg, vx, fake_slip, true);
      
      if (omg != 0 || vx != 0) {
        check_collision(period_secs);
      }
   }
}

/// \brief Helper function for publishing joint states of turtle (fake) and turtle with noise (fake_slip)
void Handler::pub_joint_state() {
  sensor_msgs::JointState js;
  std::vector<std::string> name = {left_wheel_joint, right_wheel_joint, left_wheel_joint_slip, right_wheel_joint_slip};
  std::vector<double> pos = {current_rad_left, current_rad_right, current_rad_left_slip, current_rad_right_slip};
  js.name = name;
  js.position = pos;
  joint_state_pub.publish(js);
}

/// \brief Helper function for publishing obstacle markers at ground truth locations
void Handler::pub_obst_marker() {
  visualization_msgs::MarkerArray marker_array;
  for (unsigned i = 0; i < x_coords.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "real";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_coords[i];
    marker.pose.position.y = y_coords[i];
    marker.pose.position.z = 0.5;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = obst_radius*2;
    marker.scale.y = obst_radius*2;
    marker.scale.z = 1;
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
  }
  obst_pub.publish( marker_array );
}

/// \brief Broadcast the transform from world to turtle with noise (turtle) and 
/// from world to odom. 
void Handler::tf_broadcast() {
  // world -> turtle
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "turtle";
  rigid2d::Transform2D config = fake_slip.config();
  transformStamped.transform.translation.x = config.x();
  transformStamped.transform.translation.y = config.y();
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, config.theta());
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  br.sendTransform(transformStamped);

  
  // world -> odom
  geometry_msgs::TransformStamped tmp;
  tmp.header.stamp = ros::Time::now();
  tmp.header.frame_id = "world";
  if (use_odom) {
    tmp.child_frame_id = "odom";
  }
  else {
    tmp.child_frame_id = "map";
  }
  tmp.transform.translation.x = 0.0;
  tmp.transform.translation.y = 0.0;
  tmp.transform.translation.z = 0.0;
  tmp.transform.rotation.x = 0.0;
  tmp.transform.rotation.y = 0.0;
  tmp.transform.rotation.z = 0.0;
  tmp.transform.rotation.w = 1.0;
  br.sendTransform(tmp);
}

/// \brief Helper function for publishing real_path information from fake_slip turtle 
void Handler::pub_path() {
  geometry_msgs::PoseStamped path_pose;
  path_pose.header.stamp = ros::Time::now();
  path_pose.header.frame_id = "world";
  rigid2d::Transform2D config = fake_slip.config();
  path_pose.pose.position.x = config.x();
  path_pose.pose.position.y = config.y();
  path_pose.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, config.theta());
  path_pose.pose.orientation.x = q.x();
  path_pose.pose.orientation.y = q.y();
  path_pose.pose.orientation.z = q.z();
  path_pose.pose.orientation.w = q.w();

  path_msg.poses.push_back(path_pose);
  path_pub.publish(path_msg);
}

/// \brief Helper function for publishing fake_sensor and fake_measurement information
/// Red tube marker are only shown at locations of obstacles that are within the 
/// observation range (`obst_max_dist`). 
/// fake_measurement information is an array of float in form {d1, theat_1, d2, theta_2, ...}
/// where d is the distance theta is the realtive angle to obstacles. If obstacles are out of
/// range, set distance to -1 to let Kalman filter pass calculation at this landmark. 
void Handler::pub_fake_sensor() {
  visualization_msgs::MarkerArray marker_array;
  std_msgs::Float64MultiArray float_array;
  for (unsigned i = 0; i < x_coords.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "fake";
    marker.id = i;
    marker.type = visualization_msgs::Marker::CYLINDER;

    rigid2d::Transform2D config = fake_slip.config();

    // add gaussian noise 
    std::normal_distribution<> d(0, tube_var);
    double noise_x = d(get_random());
    double noise_y = d(get_random());
    
    marker.pose.position.x = x_coords[i] + noise_x;
    marker.pose.position.y = y_coords[i] + noise_y;
    marker.pose.position.z = 0.5;

    double x_dist = x_coords[i] + noise_x - config.x();
    double y_dist = y_coords[i] + noise_y - config.y();

    // fake_measurement info
    double measure_dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));
    double angle = atan2(y_dist, x_dist) - config.theta();
    angle = rigid2d::normalize_angle(angle);
    double dist = sqrt(pow((config.x()-x_coords[i]), 2) + pow((config.y()-y_coords[i]), 2));

    // check if obstacles can be seen
    if (dist < obst_max_dist) {
      marker.action = visualization_msgs::Marker::ADD;
      float_array.data.push_back(measure_dist);
      float_array.data.push_back(angle);
    }
    else {
      marker.action = visualization_msgs::Marker::DELETE;
      float_array.data.push_back(-1);
      float_array.data.push_back(-1);
    }

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = obst_radius*2;
    marker.scale.y = obst_radius*2;
    marker.scale.z = 1;
    marker.color.a = 1.0; 
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_array.markers.push_back(marker);
  }
  fake_sensor_pub.publish( marker_array );
  fake_measurenmt_pub.publish(float_array);
}

/// \brief Helper function for collision detection between robot and obstacles.
/// After collision, this function is going to move robot along the tangent line between 
/// the robot and obstacle to move the robot away. 
void Handler::check_collision(double period_secs) {
  
  // simulate robot as a sphere 
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "fake_slip";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  rigid2d::Transform2D config = fake_slip.config();
  marker.pose.position.x = config.x();
  marker.pose.position.y = config.y();
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = robot_radius*2;
  marker.scale.y = robot_radius*2;
  marker.scale.z = robot_radius*2;
  marker.color.a = 0.5; 
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker_array.markers.push_back(marker);
  obst_pub.publish( marker_array );

  // check collision
  for (unsigned i = 0; i < x_coords.size(); i++) {
    double dist = sqrt(pow((config.x()-x_coords[i]), 2) + pow((config.y()-y_coords[i]), 2));
    if (dist <= robot_radius+obst_radius) {
      double dist_x = config.x()-x_coords[i];
      double dist_y = config.y()-y_coords[i];
      double diff_ang = atan2(dist_y, dist_x) - config.theta();
      
      dist = robot_radius+obst_radius - dist;
      dist /= period_secs;
      double angle = atan2(dist_y, dist_x);
      double sign = angle/fabs(angle);
      double tx = -sign*dist*sin(angle)/period_secs;
      double tomg = sign*0.5;

      geometry_msgs::Twist t;
      t.linear.x = tx;
      t.angular.z = tomg;
      update_turtle(period_secs, tomg, tx, fake_slip, true);
      break;
    }
  }
}


/// \brief Helper function for moving robot joint states based on commanded twist
/// \param period_secs - period of time between current and last command twist 
/// \param omg - angular velocity of robot
/// \param vx - linear velocity of robot
/// \param turtle - pointer to turlte wish to move
/// \param add_nosie - whether to add noise to commanded twist and joint states  
void Handler::update_turtle(double period_secs, double omg, double vx, rigid2d::DiffDrive & turtle, bool add_noise) {
  // add Gaussian noise 
  std::normal_distribution<> d(noise_mean, noise_variance);
  if (omg != 0 && add_noise) {
    double noise_omg = d(get_random());
    omg = omg + noise_omg;
  }
  if (vx != 0 && add_noise) {
    double noise_vx = d(get_random());
    vx = vx + noise_vx;
  }

  omg = omg*period_secs;
  vx = vx*period_secs;

  rigid2d::Twist2D t {omg, vx, 0};
  rigid2d::DiffDriveVel wheel_vel = turtle.vel_from_twist(t);

  // add uniform random noise
  std::uniform_real_distribution<double> d_slip(slip_min, slip_max);
  double vL_slip = wheel_vel.vL;
  double vR_slip = wheel_vel.vR;
  if (vL_slip != 0 && add_noise) {
    vL_slip *= d_slip(get_random()) + 1;
  }      
  if (vR_slip != 0 && add_noise) {
    vR_slip *= d_slip(get_random()) + 1;
  }
  turtle.update(vL_slip, vR_slip);

  if (add_noise == false) {
    current_rad_left += vL_slip;
    current_rad_right += vR_slip;
  }
  else {
    current_rad_left_slip += vL_slip;
    current_rad_right_slip += vR_slip;
  }

}
    
void Handler::timer_callback(const ros::TimerEvent& event){
  sensor_msgs::LaserScan laser_scan_msg;
  laser_scan_msg.header.stamp = ros::Time();
  laser_scan_msg.header.frame_id = "turtle";
  laser_scan_msg.angle_min = 0.0;
  laser_scan_msg.angle_max = rigid2d::PI * 2;
  laser_scan_msg.angle_increment = laser_angle_increment;
  laser_scan_msg.range_min = laser_range_min;
  laser_scan_msg.range_max = laser_range_max;

  rigid2d::Transform2D config = fake_slip.config();

  arma::vec r_arr (laser_samples_num);
  r_arr.fill(laser_range_max+1);

  arma::vec x_arr (laser_samples_num);
  arma::vec y_arr (laser_samples_num);
  for (unsigned i = 0; i < laser_samples_num; i ++) { // make walls
    double angle = i * laser_angle_increment;
    angle = fmod(angle, rigid2d::PI/2);

    double r, x, y;
    if (angle <= rigid2d::PI/4){
      r = wall_size/cos(angle);
    }
    else {
      r = wall_size/sin(angle);
    }

    if (i <= 90) {
      x = r * cos(angle);
      y = r * sin(angle);
    }
    else if (i <= 180) {
      x = -r * sin(angle);
      y = r * cos(angle);
    } 
    else if (i <= 270) {
      x = -r * cos(angle);
      y = -r * sin(angle);
    } 
    else if (i <= 360) {
      x = r * sin(angle);
      y = -r * cos(angle);
    } 

    // r = sqrt(pow(x, 2)+pow(y,2));
    // r_arr(i) = r;
    x_arr(i) = x;
    y_arr(i) = y;
    // ROS_INFO_STREAM("Check x y " << i << " --- " << r << " " << x << " " << y );
  }

  for (unsigned i = 0; i < laser_samples_num; i ++) { // convert to turtle frame 
    double mx = x_arr(i);
    double my = y_arr(i);
    double x = mx - config.x();
    double y = my - config.y();

    double angle = atan2(y, x) - config.theta();
    angle = rigid2d::normalize_angle(angle);
    if (angle < 0) {
      angle += rigid2d::PI * 2;
    } 
    int angle_idx = angle / laser_angle_increment;
    angle_idx = fmod(angle_idx, 360);

    double r = sqrt(pow(x,2)+pow(y,2));
    // ROS_INFO_STREAM("Check r " << angle_idx << " --- " << r << " " << x << " " << y);

    for (unsigned j = 0; j < x_coords.size(); j++) { // check landmarks 
      double x_dist = x_coords[j] - config.x();
      double y_dist = y_coords[j] - config.y();
      double measure_dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));
      double laser_angle = atan2(y_dist, x_dist) - config.theta();
      laser_angle = rigid2d::normalize_angle(laser_angle);
      if (laser_angle < 0) {
        laser_angle += rigid2d::PI * 2;
      } 

      if (rigid2d::almost_equal(angle, laser_angle, 0.1)) {
        // r = measure_dist - 1.2 * obst_radius;
        // r *= (1 + 10*pow(fabs(laser_angle - angle),2));
        // r *= (1 + fabs(laser_angle - angle));

        rigid2d::Vector2D v_wall {mx, my};
        rigid2d::Transform2D T_wall {v_wall};
        rigid2d::Vector2D v_land {x_coords[j], y_coords[j]};
        rigid2d::Transform2D T_land {v_land};

        rigid2d::Transform2D T_land_wall = T_land.inv() * T_wall;
        rigid2d::Transform2D T_land_turtle = T_land.inv() * config;
        // ROS_INFO_STREAM("wall =  " << T_wall << " --- land = " << T_land);
        // ROS_INFO_STREAM("T_land_wall " << T_land_wall);
        // ROS_INFO_STREAM("T_land_turtle " << T_land_turtle);

        double x1 = T_land_wall.x();
        double y1 = T_land_wall.y();
        double x2 = T_land_turtle.x();
        double y2 = T_land_turtle.y();

        double dx = x2-x1;
        double dy = y2-y1;
        double dr = sqrt(pow(dx,2)+pow(dy,2));
        double D = x1*y2 - x2*y1;
        double discri = pow(obst_radius,2)*pow(dr,2) - pow(D,2);
        // ROS_INFO_STREAM("discri = " << discri);

        if (discri > 0) {
          int sign = 1;
          if (dy < 0) {
            sign = -1;
          }

          double inters_x_1 = (D * dy + sign * dx * sqrt(discri))/pow(dr,2);
          double inters_x_2 = (D * dy - sign * dx * sqrt(discri))/pow(dr,2);
          double inters_y_1 = (-D * dx + fabs(dy) * sqrt(discri))/pow(dr,2);
          double inters_y_2 = (-D * dx - fabs(dy) * sqrt(discri))/pow(dr,2);
          // ROS_INFO_STREAM("intersect = " << inters_x_1 << " " << inters_y_1 << " OR " << inters_x_2 << " " << inters_y_2);

          rigid2d::Vector2D v1 {inters_x_1, inters_y_1};
          rigid2d::Transform2D T1 {v1};
          rigid2d::Vector2D v2 {inters_x_2, inters_y_2};
          rigid2d::Transform2D T2 {v2};

          rigid2d::Transform2D T_turtle_1 = T_land_turtle.inv() * T1;
          rigid2d::Transform2D T_turtle_2 = T_land_turtle.inv() * T2;
          // ROS_INFO_STREAM("T1 " << T_turtle_1);
          // ROS_INFO_STREAM("T2 " << T_turtle_2);

          double r1 = sqrt(pow(T_turtle_1.x(),2)+pow(T_turtle_1.y(),2));
          double r2 = sqrt(pow(T_turtle_2.x(),2)+pow(T_turtle_2.y(),2));
          r = fmin(r1, r2);

        }

        break;
      }
    }

    r_arr(angle_idx) = r;
  }

  for (unsigned i = 0; i < laser_samples_num; i ++) { // publish 
    std::normal_distribution<> d(0, laser_noise_level);
    double noise_r = d(get_random());
    double r = r_arr(i);
    if (r > laser_range_max) {
      int idx = i - 1;
      if (idx < 0) {
        idx += laser_samples_num;
      }
      // ROS_INFO_STREAM(idx);
      r = r_arr(idx);
    }
    laser_scan_msg.ranges.push_back(r+noise_r);
  }

  laser_scan_pub.publish(laser_scan_msg);
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "tube_world");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}

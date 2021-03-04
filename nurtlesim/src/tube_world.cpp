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
    double slip_min = -0.05;
    double slip_max = 0.05;
    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::vector<std::vector<double>> covariance_matrix = {{1.0, 1.0}, {1.0, 1.0}};;
    double obst_radius = 0.0762;
    double obst_max_dist = 2.0;
    double tube_var = 0.1;
    rigid2d::DiffDrive fake;             
    rigid2d::DiffDrive fake_slip;        
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

  
  while (ros::ok()) {
    find_param(n);

    pub_joint_state();
    pub_obst_marker();
    pub_path();
    tf_broadcast();
    pub_fake_sensor();


    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
  }
    
}

/// \brief Helper function for generating random gaussian noise
/// \return A random generator 
std::mt19937 & Handler::get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

/// \brief Find parameters wheel_base, and wheel_radius, left_wheel_joint, and right_wheel_joint
/// from ROS parameter server, otherwise set to default value
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
}

/// \brief Callback function for cmd_vel subscriber
/// Compute wheel velocities from given twist (cmd_vel), then update turtle config
/// and wheel angles. 
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

/// \brief Helper function for publishing joint states
void Handler::pub_joint_state() {
  sensor_msgs::JointState js;
  std::vector<std::string> name = {left_wheel_joint, right_wheel_joint, left_wheel_joint_slip, right_wheel_joint_slip};
  std::vector<double> pos = {current_rad_left, current_rad_right, current_rad_left_slip, current_rad_right_slip};
  js.name = name;
  js.position = pos;
  joint_state_pub.publish(js);

}

/// \brief Helper function for publishing obstacle markers
void Handler::pub_obst_marker() {
  visualization_msgs::MarkerArray marker_array;
  for (unsigned i = 0; i < x_coords.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
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

/// \brief Broadcast the transform between odom and the turtle
void Handler::tf_broadcast() {
  // world -> turtle
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "turtle";
  rigid2d::Transform2D config = fake_slip.config();
  // ROS_INFO_STREAM("turtle " << config.theta() << " " << config.x() << " " << config.y());
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

  // tmp world -> odom
  geometry_msgs::TransformStamped tmp;
  tmp.header.stamp = ros::Time::now();
  tmp.header.frame_id = "world";
  tmp.child_frame_id = "odom";
  tmp.transform.translation.x = 0.0;
  tmp.transform.translation.y = 0.0;
  tmp.transform.translation.z = 0.0;
  tmp.transform.rotation.x = 0.0;
  tmp.transform.rotation.y = 0.0;
  tmp.transform.rotation.z = 0.0;
  tmp.transform.rotation.w = 1.0;
  br.sendTransform(tmp);
}

/// \brief publish real_path information from fake_slip turtle 
void Handler::pub_path() {
  geometry_msgs::PoseStamped path_pose;
  path_pose.header.stamp = ros::Time::now();
  path_pose.header.frame_id = "odom";
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

/// \brief Helper function for publishing fake sensor information
void Handler::pub_fake_sensor() {
  visualization_msgs::MarkerArray marker_array;
  std_msgs::Float64MultiArray float_array;
  for (unsigned i = 0; i < x_coords.size(); i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
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
    // ROS_INFO_STREAM("@ tube world (x,y) = " << x_dist << " " << y_dist);

    double measure_dist = sqrt(pow(x_dist, 2) + pow(y_dist, 2));
    double angle = atan2(y_dist, x_dist) - config.theta();
    angle = rigid2d::normalize_angle(angle);

    double dist = sqrt(pow((config.x()-x_coords[i]), 2) + pow((config.y()-y_coords[i]), 2));
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

    // ROS_INFO_STREAM("@ tube world config = " << config.theta() << " " << config.x() << " " << config.y());
    // ROS_INFO_STREAM("@" << i << "tube world z = " << measure_dist<< " " << angle);

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
  rigid2d::Transform2D config_slip = fake_slip.config();
  // ROS_INFO_STREAM("turtle: " << config_slip.theta() << " " << config_slip.x() << " " << config_slip.y());
  
  // simulate robot as a sphere 
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
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
      double sign = angle/abs(angle);
      double tx = -sign*dist*sin(angle)/period_secs;
      // double ty = -sign*dist*cos(angle)/period_secs;
      // double tomg = diff_ang*dist;
      double tomg = sign*0.5;
      // ROS_INFO_STREAM("dist " << dist << " angle " << angle << " --> tangent: " << tx << " " << ty << " " << tomg);

      geometry_msgs::Twist t;
      t.linear.x = tx;
      t.angular.z = tomg;
      update_turtle(period_secs, tomg, tx, fake_slip, true);
      break;
    }
  }
}

void Handler::update_turtle(double period_secs, double omg, double vx, rigid2d::DiffDrive & turtle, bool add_noise) {
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
    ROS_INFO_STREAM(" ---- update js " << current_rad_left << "  " << current_rad_right);
  }
  else {
    current_rad_left_slip += vL_slip;
    current_rad_right_slip += vR_slip;
    ROS_INFO_STREAM(" ---- update js slip " << current_rad_left << "  " << current_rad_right);
  }

  rigid2d::Transform2D config_slip = fake_slip.config();
  ROS_INFO_STREAM(" --- fake slip " << config_slip.theta() << " "<< config_slip.x() << " " << config_slip.y());
  rigid2d::Transform2D config = fake.config();
  ROS_INFO_STREAM(" --- fake  " << config.theta() << " "<< config.x() << " " << config.y());
  // ROS_INFO_STREAM("command cmd vel = " << omg << " " << vx);
  // ROS_INFO_STREAM(period_secs <<" fake slip " << config_slip.x() << " fake " << config.x());
  // ROS_INFO_STREAM(period_secs <<" fake slip " << config_slip.theta() << " fake " << config.theta());
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "tube_world");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}

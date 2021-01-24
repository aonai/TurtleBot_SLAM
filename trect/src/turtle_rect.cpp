/**
 * \brief This node is going to a turtle in turtlesim follow a rectangular trajectory after 
 * calling start service
 * 
 * PUBLISHERS:
 * + cmd_vel (geometry_msgs::Twist) ~ the velocity of turtle
 * 
 * SUBSRIBERS:
 * + pose (turtlesim::PoseConstPtr) ~ pose of turtle 
 * 
 * SERVICES:
 * + start (trect::Start) ~ make turtle start following trajectory
**/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "trect/Start.h"
#include "trect/StartMsg.h"
#include <sstream>


/// \brief Helper class for class turtle_rect 
class Handler {
  public:
    explicit Handler(ros::NodeHandle & n);
  
  private:
    // variables 
    double PI=3.14159265358979323846;
    double max_xdot = 2.0;
    double max_wdot = 1.0;
    int frequency = 100;
    double currentX = 0.0;
    double currentY = 0.0;
    double currentTheta = 0.0;
    double xdot = 0;
    double wdot = 0;
    double rect [4][3];
    bool following = false;
    int sideIndex = 0;
    // ROS members
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::ServiceServer service;
    ros::ServiceClient clear;
    ros::ServiceClient teleAbs;
    // helper functions
    void find_param(ros::NodeHandle & n);
    bool approx_eq(double num1, double num2, double tol);
    bool check_pose(double nextX, double nextY);
    void draw_rect();
    void follow_rect();
    void sub_callback(const turtlesim::PoseConstPtr & pose);
    bool service_callback(trect::Start::Request  & req, trect::Start::Response & res);
};

/// \brief Init Handler class
/// \param n - turtle_rect NodeHandle
Handler::Handler(ros::NodeHandle & n) {
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  sub = n.subscribe("pose", 10, &Handler::sub_callback, this);
  service = n.advertiseService("start", &Handler::service_callback, this);
  clear = n.serviceClient<std_srvs::Empty>("clear");
  teleAbs = n.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute");
  
  while (ros::ok()) {
    find_param(n);

    if (following) {
      follow_rect();
    }
    
    ros::Rate loop_rate(frequency);
    
    // publish cmd_vel
    geometry_msgs::Twist msg;
    msg.linear.x = xdot;
    msg.angular.z = wdot;
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

/// \brief Find parameters max_xdot, max_wdot, and frequency from ROS parameter server, 
/// otherwise set to default value
/// \param n - turtle_rect NodeHandle
void Handler::find_param(ros::NodeHandle & n) {
  n.param("max_xdot", max_xdot, 2.0);
  n.param("max_wdot", max_wdot, 1.0);
  n.param("frequency", frequency, 100);
}

/// \brief Check if num1 and num2 are approximately equal within tolerance 
/// \param num1 - first number to compare
/// \param num2 - second number to compare
/// \param tol - tolerance 
/// \returns true if num1 ~= num2
bool Handler::approx_eq(double num1, double num2, double tol) {
  if (abs(num1 - num2) < tol) {
      return true;
  }
  return false;
}

/// \brief Check turtle is close to corner 
/// \param nextX - x of corner
/// \param nextY - y of corner
/// \returns true if turtle has reached corner
bool Handler::check_pose(double nextX, double nextY) {
  if (sideIndex == 0) {
    if (approx_eq(currentY, nextY, 0.01) || currentY > nextY) {
      return true;
    }
  }
  else if (sideIndex == 1) {
    if (approx_eq(currentX, nextX, 0.01) || currentX > nextX) {
      return true;
    }
  }
  else if (sideIndex == 2) {
    if (approx_eq(currentY, nextY, 0.01) || currentY < nextY) {
      return true;
    }
  }
  else if (sideIndex == 3) {
    if (approx_eq(currentX, nextX, 0.01) || currentX < nextX) {
      return true;
    }
  }

  return false;
}

/// \brief Helper function for drawing rectangluar trajectory 
void Handler::draw_rect() {
  // teleport to start of trajectory
  turtlesim::TeleportAbsolute teleMsg;
  teleMsg.request.x = rect[0][0];
  teleMsg.request.y = rect[0][1];
  teleMsg.request.theta = PI/2;
  teleAbs.call(teleMsg);

  // clear background
  std_srvs::Empty empty;
  clear.call(empty);
  
  // start drawing
  for (int i = 1; i < 5; i++) {
    if (i == 4) { i = 0; }
    teleMsg.request.x = rect[i][0];
    teleMsg.request.y = rect[i][1];
    teleAbs.call(teleMsg);
    if (i == 0) { break; }
  }
}

/// \brief Helper function for making turtle following the rectangular trajectory drawn
void Handler::follow_rect() {
  xdot = 0;
  wdot = 0;
  int next = sideIndex + 1;
  if (next == 4) { next = 0; }
  double nextX = rect[next][0];
  double nextY = rect[next][1];
  double nextTheta = rect[next][2];
  
  if (approx_eq(currentTheta, nextTheta, 0.01) == false) {
    // turn at corner  
    xdot = 0;
    if (currentTheta > nextTheta) {
      wdot = -1*max_wdot;
    }
    else {
      wdot = max_wdot;
    }
  }
  else if (check_pose(nextX, nextY) == false) { 
    // move forward
    xdot = max_xdot;
    wdot = 0;
  }
  else {
    if (check_pose(nextX, nextY) && approx_eq(currentTheta, nextTheta, 0.01)) {
      // follow next side
      sideIndex += 1;
      if (sideIndex == 4) {
        sideIndex = 0;
      }
    }
  }
}

/// \brief Callback function for pose subscriber
/// \param pose - current pose of turtle
void Handler::sub_callback(const turtlesim::PoseConstPtr & pose) {
  /** Callback function for pose subscriber **/
  currentX = pose->x;
  currentY = pose->y;
  currentTheta = pose->theta;
  ROS_INFO("x = %f, y = %f,, theta = %f", currentX, currentY, currentTheta);
}

/// \brief Callback function for start service 
/// \param req - request of start service 
/// \param res - responce of start service 
/// \returns true if service call is success and complete
bool Handler::service_callback(trect::Start::Request & req, trect::Start::Response & res) {
  // store trajectory
  double x = req.msg.fromX;
  double y = req.msg.fromY;
  double w = req.msg.width;
  double h = req.msg.height;
  rect[0][0] = x; 
  rect[0][1] = y;
  rect[0][2] = PI-0.01; // add some tolerence to avoid struggling at PI and -PI
  rect[1][0] = x; 
  rect[1][1] = y+h;
  rect[1][2] = PI/2;
  rect[2][0] = x+w; 
  rect[2][1] = y+h;
  rect[2][2] = 0;
  rect[3][0] = x+w; 
  rect[3][1] = y;
  rect[3][2] = -PI/2;
  
  // draw and start follow rectangular trajectory
  draw_rect();
  following = true;
  sideIndex = 0;
    
  return true;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "turtle_rect");
  ros::NodeHandle n;
  Handler handler(n);
  return 0;
}
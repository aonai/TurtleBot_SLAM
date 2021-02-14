#include <iostream>
#include <limits>
#include <cmath>
#include <cstring>
#include "rigid2d/diff_drive.hpp"

namespace rigid2d {
    DiffDrive::DiffDrive(double base, double radius) : wheel_base(base), wheel_radius(radius) {}

    Transform2D DiffDrive::config() const {
        return T_config;
    }
    
    DiffDriveVel DiffDrive::vel_from_twist(Twist2D t) {
        double vel_left = (1/wheel_radius)*(-wheel_base*t.omg + t.vx);   // equation (1) of Kinematics.pdf 
        double vel_right = (1/wheel_radius)*(wheel_base*t.omg + t.vx);   // equation (2) of Kinematics.pdf 
        DiffDriveVel vel {vel_left, vel_right};
        return vel;
    } 
    
    void DiffDrive::update(double rad_left, double rad_right) {
        double omg = 0.5*(rad_right-rad_left)*wheel_radius/wheel_base;   // equation (4) of Kinematics.pdf 
        double vx = rad_left*wheel_radius + wheel_base*omg;              // equation (3) of Kinematics.pdf 
        Twist2D t {omg, vx, 0};
        Transform2D T;
        T_config *= T.integrateTwist(t);
    }

    void DiffDrive::set_pose(double theta, double x, double y) {
        Vector2D newV {x, y};
        Transform2D new_config(newV, theta);
        T_config = new_config;
    }

}

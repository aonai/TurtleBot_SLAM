#include <iostream>
#include <limits>
#include <cmath>
#include <cstring>
#include "../include/rigid2d/diff_drive.hpp"

namespace rigid2d {
    DiffDrive::DiffDrive(double base, double radius) : wheel_base(base), wheel_radius(radius) {
        H[0][0] = -base/radius;
        H[1][0] = base/radius;
        H[0][1] = 1;
        H[1][1] = 1;
        H[0][2] = 0;
        H[1][2] = 0;
    }

    Transform2D DiffDrive::config() const {
        return T_config;
    }
    
    DiffDriveVel DiffDrive::vel_from_twist(Twist2D t) {
        double vel_left = this->H[0][0]*t.omg + this->H[0][1]*t.vx;    
        double vel_right = this->H[1][0]*t.omg + this->H[1][1]*t.vx;    
        DiffDriveVel vel {vel_left, vel_right};
        return vel;
    } 
    
    void DiffDrive::update(double rad_left, double rad_right) {
        double omg = 0.5*(rad_right-rad_left)*wheel_radius/wheel_base;
        double vx = rad_left + wheel_base*omg/wheel_radius;
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

#ifndef DIFF_DRIVE_GUARD_HPP
#define DIFF_DRIVE_GUARD_HPP
/// \file
/// \brief Library for differential drive robot kinematics. Refer to doc/Kinematics.pdf for mathematical calculations.

#include<iosfwd> 
#include "rigid2d.hpp"

namespace rigid2d
{
   /// \brief left and right velocities of a diff-drive robot
    struct DiffDriveVel
    {
        double vL = 0.0;
        double vR = 0.0;
    };


    /// \brief kinematic operations for a differential-drive robot
    class DiffDrive
    {
    private:
        double wheel_base = 0.0;
        double wheel_radius = 0.0;
        Transform2D T_config;

    public:
        /// \brief Create a kinematics model for a diff drive robot
        /// with the given base and radius.
        /// \param base - wheel base dimension
        /// \param radius - wheel radius dimension
        DiffDrive(double base, double radius);
        
        /// \brief get the current config of the robot
        /// \return current configuration as a Transform2D
        Transform2D config() const;
        
        /// \brief Convert a desired twist to the equivalent wheel 
        /// velocities required to achieve that twist
        /// \param t - twist to achieve
        /// \return velocities to achieve desired twist
        DiffDriveVel vel_from_twist(Twist2D t);   
        
        /// \brief update config of the robot given relative wheel angles
        /// to current body frame of robot 
        /// \param rad_left - updated left wheel angle in radian
        /// \param rad_right - updated right wheel angle in radian
        void update(double rad_left, double rad_right);

        /// \brief set config of the robot given an new configuration
        /// \param theta - theta of the new config in radian
        /// \param x - theta of the new config
        /// \param y - theta of the new config
        void set_pose(double theta, double x, double y);

        Twist2D twist_from_vel(DiffDriveVel vel);
        
    };
}

#endif

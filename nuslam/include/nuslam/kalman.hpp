#ifndef KALMAN_GUARD_HPP
#define KALMAN_GUARD_HPP

#include <armadillo>
#include "rigid2d/rigid2d.hpp"

namespace kalman {
    /// \brief Combined state vector for robot state and map state
    /// robot state qt = [theta_t, x_t, y_t].T
    /// map state mt = [m_x1, m_y1, ..., m_xn, m_yn].T, where n is the number of obstacles
    /// This class includes functions to update states using extended Kalman Filter Slam algorithm.
    /// Detailed of Kalman slam can be found on https://nu-msr.github.io/navigation_site/slam.pdf
    class StateVec {
        private:
            arma::vec robot_state;
            arma::vec map_state;
            arma::mat cov_mat;
            double state_noise_var;
            double sensor_noise_var; 
            double angle_idx = 0;
            double x_idx = 1;
            double y_idx = 2;
        
        public:
            /// \brief Create a state with 0 robot states and given map states.
            /// \param map_state - known obstacle locations
            /// \param state_noise_var - Gaussian noise variance in states
            /// \param sensor_noise_var - Gaussian noise variance in sensor measurements
            StateVec(arma::vec map_state, double state_noise_var, double sensor_noise_var);

            /// \brief Create a state with given robot states and map states.
            /// \param robot_state - knwon robot config
            /// \param map_state - kown obstacle locations
            /// \param state_noise_var - Gaussian noise variance in states
            /// \param sensor_noise_var - Gaussian noise variance in sensor measurements
            StateVec(arma::vec robot_state, arma::vec map_state, double state_noise_var, double sensor_noise_var);

            /// \brief Update robot state based on a twist command 
            /// Robot state are estimated values.
            /// Map state will remain constant.
            /// Equation (5) and (7) of slam.pdf
            /// \param t - commanded twist
            /// \return a matrix for derivative of state transition
            arma::mat estimate(rigid2d::Twist2D t);

            /// \brief Compute the derivative of state transition with respect to state 
            /// based on a twist command
            /// Equation (9) and (10) of slam.pdf
            /// \param t - commanded twist
            /// \return a matrix for derivative of state transition
            arma::mat A_mat(rigid2d::Twist2D t);

            /// \brief Compute measurement model relative to an obstacle
            ///       relative x = obstacle_x - robot_x
            ///       relative y = obstacle_y - robot_y
            ///       relative angle = atan2(rel_y, rel_x) - robot_angle
            /// Equation (14) of slam.pdf
            /// \param j - index of obstacle to measure 
            /// \return - vector in form [relative distance, relative angle].T
            arma::vec measurement_vec(unsigned j);

            /// \brief Compute the derivative of measurement model with respect to sate 
            /// Equation (18) of slam.pdf
            /// \param j - index of obstacle to measure 
            /// \return a matrix for derivative of measurement model
            arma::mat H_mat(unsigned j);

            /// \brief Update covariance matrix 
            /// Equation (21) of slam.pdf
            /// \param A - matrix for derivative of state transition when updating 
            ///             robot state based on a twist command
            void update_cov(arma::mat A); 

            /// \brief Calculate the Kalman gain from linearized measurement model
            /// Equation(26) of slam.pdf
            /// \param H - matrix for derivative of measurement model at some index
            /// \return Kalman gain
            arma::mat K_mat(arma::mat H);

            /// \brief Update states using extended Kalman Filter slam algorithm 
            /// Section 3.3 Update of slam.pdf 
            /// \param t - commanded twist
            /// \param actual_d - measured distance to obstacles
            /// \param actual_angle - measured relative angle to obstacles 
            /// -1 in measurement indicate that obstacle is not seen by the robot, so
            /// map state will not be updated at these index.  
            void ekf_update(rigid2d::Twist2D t, arma::vec actual_d, arma::vec acutal_angle);

            /// \brief Get robot state of model
            /// \return robot state
            arma::vec get_robot_state();

            /// \brief Get map state of model
            /// \return map state
            arma::vec get_map_state();
            
            /// \brief Set map state of model
            /// \param ms - new map state
            void set_map_state(arma::vec ms);

            /// \brief Set robot state of model
            /// \param rs - new robot state 
            void set_robot_state(arma::vec rs);

            /// \brief Reset covariance matrix
            /// Equation (22)
            void reset_cov();

            /// \brief Set noise variance of model
            /// \param state_nv - Gaussian noise variance in states
            /// \param sensor_nv - Gaussian noise variance in sensor measurement 
            void set_noises(double state_nv, double sensor_nv);
            
    };


    /// \brief Helper function for generating random gaussian noise
    /// \return A random generator 
    std::mt19937 & get_random();

    /// \brief Generate a vector of gaussian noises 
    /// \param variance - variance of gaussian noise
    /// \param n - number of noises to generate 
    /// \return a vector containing the noises 
    arma::vec gau_noise(double variance, unsigned n);

};

#endif

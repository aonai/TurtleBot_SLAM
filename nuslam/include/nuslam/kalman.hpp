#ifndef KALMAN_GUARD_HPP
#define KALMAN_GUARD_HPP

#include <armadillo>
#include "rigid2d/rigid2d.hpp"

namespace kalman {
    /// \brief Combined state vector for robot state and map state
    /// robot state qt = [theta_t, x_t, y_t].T
    /// map state mt = [m_x1, m_y1, ..., m_xn, m_yn].T, where n is the number of obstacles
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
            /// \brief Create a state with 0 robot states and map states.
            /// \param n_obst - number of obstacles
            StateVec(arma::vec map_state, double state_noise_var, double sensor_noise_var);

            /// \brief Create a state with given robot states and map states.
            /// \param robot_state - robot state
            /// \param map_state - map state
            /// \param cov - covariance matrix
            StateVec(arma::vec robot_state, arma::vec map_state, arma::mat cov_mat, double state_noise_var, double sensor_noise_var);

            /// \brief Update robot state based on a twist command 
            /// Robot state are estimated values.
            /// Map state will remain constant.
            /// Equation (5) and (7) of slam.pdf
            /// \param t - commanded twist
            void estimate(rigid2d::Twist2D t);

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
            /// \param t - commanded twist
            void update_cov(rigid2d::Twist2D t); 

            /// Calculate the Kalman gain from linearized measurement model
            /// Equation(26)
            /// \param j - index of obstacle to measure 
            arma::mat K_mat(unsigned j);

            void ekf_update(rigid2d::Twist2D t, double actual_d, double acutal_angle);

            
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

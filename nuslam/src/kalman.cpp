#include <iostream>
#include <cmath>
#include "nuslam/kalman.hpp"

namespace kalman {

    StateVec::StateVec(arma::vec map_state, double state_noise_var, double sensor_noise_var) : 
        robot_state(arma::vec(3, arma::fill::zeros)), 
        map_state(map_state),
        state_noise_var(state_noise_var),
        sensor_noise_var(sensor_noise_var)
    {   
        arma::mat robot_cov (3, 3,  arma::fill::zeros);
        unsigned n_obst = map_state.size()/2;
        arma::mat map_cov (2*n_obst, 2*n_obst,  arma::fill::zeros);

        for (unsigned i = 0; i < 2*n_obst; i++) {
            map_cov(i, i) = 1000;
        }

        unsigned n = 3 + 2*n_obst;
        cov_mat = arma::mat(n, n, arma::fill::zeros);
        
        for (unsigned r = 0; r < 3; r++) {
            for (unsigned c = 0; c < 3; c++) {
                cov_mat(r, c) = robot_cov(r, c);
            }
        }

        for (unsigned r = 3; r < n; r++) {
            for (unsigned c = 3; c < n; c++) {
                cov_mat(r, c) = map_cov(r-3, c-3);
            }
        }

        std::cout << "cov " << cov_mat << std::endl;
        std::cout << "robot " << robot_state << std::endl;
        std::cout << "map " << map_state << std::endl;
    }

    StateVec::StateVec(arma::vec robot_state, arma::vec map_state, arma::mat cov_mat, double state_noise_var, double sensor_noise_var):
        robot_state(robot_state), 
        map_state(map_state),
        cov_mat(cov_mat),
        state_noise_var(state_noise_var),
        sensor_noise_var(sensor_noise_var)
    {   
        robot_state(angle_idx) = rigid2d::normalize_angle(robot_state(angle_idx));
        std::cout << "cov " << cov_mat << std::endl;
        std::cout << "robot = " << robot_state << std::endl;
        std::cout << "map = " << map_state << std::endl;
        std::cout << "state noise = " << state_noise_var << std::endl;
        std::cout << "sensor noise = " << sensor_noise_var << std::endl;
    }

    void StateVec::estimate(rigid2d::Twist2D t) {
        arma::vec noises = gau_noise(state_noise_var, 3);
        // arma::vec noises {0, 0, 0};

        if (t.omg == 0) {  // Equation (5)
            double new_angle = robot_state(angle_idx) + noises(angle_idx);
            new_angle = rigid2d::normalize_angle(new_angle);
            double new_x = robot_state(x_idx) + t.vx * cos(robot_state(angle_idx)) + noises(x_idx);
            double new_y = robot_state(y_idx) + t.vx * sin(robot_state(angle_idx)) + noises(y_idx);

            arma::vec new_robot_state {new_angle, new_x, new_y};
            robot_state = new_robot_state;
        }
        else { // Equation (7)
            double new_angle = robot_state(angle_idx) + t.omg + noises(angle_idx);
            new_angle = rigid2d::normalize_angle(new_angle);
            double new_x = robot_state(x_idx) - (t.vx/t.omg)*sin(robot_state(angle_idx)) \
                          + (t.vx/t.omg)*sin(robot_state(angle_idx)+t.omg) + noises(x_idx);
            double new_y = robot_state(y_idx) + (t.vx/t.omg)*cos(robot_state(angle_idx)) \
                          - (t.vx/t.omg)*cos(robot_state(angle_idx)+t.omg) + noises(y_idx);

            arma::vec new_robot_state {new_angle, new_x, new_y};
            robot_state = new_robot_state;
        }
    }

    arma::mat StateVec::A_mat(rigid2d::Twist2D t) {
        unsigned n = 3 + map_state.size();         
        arma::mat A(n, n, arma::fill::eye);

        if (t.omg == 0) { // Equation (9)
            A(1, 0) += -t.vx * sin(robot_state(angle_idx));
            A(2, 0) += t.vx * cos(robot_state(angle_idx));
        }
        else { // Equation (10)
            A(1, 0) += -(t.vx/t.omg)*cos(robot_state(angle_idx)) + (t.vx/t.omg)*cos(robot_state(angle_idx)+t.omg);
            A(2, 0) += -(t.vx/t.omg)*sin(robot_state(angle_idx)) + (t.vx/t.omg)*sin(robot_state(angle_idx)+t.omg);
        }

        return A;
    }

    arma::vec StateVec::measurement_vec(unsigned j) {
        arma::vec m(2);
        
        double dist_x = map_state(j*2) - robot_state(x_idx);
        double dist_y = map_state(j*2+1) - robot_state(y_idx);

        m(0) = sqrt(pow(dist_x,2) + pow(dist_y,2));
        m(1) = atan2(dist_y, dist_x) - robot_state(angle_idx);
        std::cout << "measurement = " << m.t() << std::endl;
        return m;
    }

    arma::mat StateVec::H_mat(unsigned j) {
        unsigned n = 3 + map_state.size();         
        arma::mat H(2, n, arma::fill::zeros);

        double dist_x = map_state(j*2) - robot_state(x_idx);
        double dist_y = map_state(j*2+1) - robot_state(y_idx);
        double d = pow(dist_x,2) + pow(dist_y,2);

        H(1, 0) = -1;
        H(0, 1) = -dist_x/sqrt(d);
        H(1, 1) = dist_y/d;
        H(0, 2) = -dist_y/sqrt(d);
        H(1, 2) = -dist_x/d;

        unsigned i = 3 + 2*j;
        H(0, i) = dist_x/sqrt(d);
        H(1, i) = -dist_y/d;

        i += 1;
        H(0, i) = dist_y/sqrt(d);
        H(1, i) = dist_x/d;

        std::cout << "H = " << H << std::endl;
        return H;
    }

    void StateVec::update_cov(rigid2d::Twist2D t) {
        arma::mat A = A_mat(t);
        std::cout << "A mat " << A << std::endl;
        std::cout << "state noise var = " << state_noise_var << std::endl;
        
        unsigned n = 3 + map_state.size();
        arma::mat Q (n, n, arma::fill::zeros);
        for (unsigned r  = 0; r < 3; r++) {
            for (unsigned c= 0; c < 3; c++) {
                Q(r,c) = state_noise_var;
            }
        }

        arma::mat new_cov_mat;
        new_cov_mat = A * cov_mat * A.t() + Q;
        cov_mat = new_cov_mat;
    }
    
    arma::mat StateVec::K_mat(unsigned j) {
        arma::mat H = H_mat(j);
        arma::mat R (2,2);
        R.fill(sensor_noise_var);

        arma::mat K (3+map_state.size(), 2);
        K = H * cov_mat * H.t() + R;
        K = cov_mat * H.t() * arma::inv(K);
        std::cout << "K = " << K << std::endl;

        return K;
    }


    void StateVec::ekf_update(rigid2d::Twist2D t, double actual_d, double acutal_angle) {
        std::cout << "--------- start ----------- " << std::endl;

        std::cout << "--------- update ----------- " << std::endl;
        estimate(t);
        std::cout << "update robot = " << robot_state << std::endl;
        update_cov(t);
        std::cout << "update cov = " << cov_mat << std::endl;

        for (unsigned obst_idx = 0; obst_idx < map_state.size()/2; obst_idx++) {
            std::cout << "--------- 1 ----------- " << std::endl;
            arma::vec z = measurement_vec(obst_idx);       // Equation (25)   
            std::cout << "--------- 2 ----------- " << std::endl;
            arma::mat K = K_mat(obst_idx);                 // Equation (26)
            std::cout << "--------- 3 ----------- " << std::endl;
            arma::vec z_act (2,arma::fill::zeros);
            z_act(0) = actual_d ;
            z_act(1) = acutal_angle;
            arma::vec z_diff = z_act - z;
            std::cout << "diff z = " << z_diff << std::endl;
            arma::vec tmp = K * z_diff;
            robot_state(0) = robot_state(0) + tmp(0);
            robot_state(1) = robot_state(1) + tmp(1);
            robot_state(2) = robot_state(2) + tmp(2);

            for (unsigned i =0 ; i < map_state.size(); i++) {
                map_state(i) = map_state(i) + tmp(3+i);
            }

            std::cout << "posterior robot = " << robot_state << std::endl;
            std::cout << "posterior map = " << map_state << std::endl;
            arma::mat tmp2 (arma::size(cov_mat), arma::fill::eye);
            tmp2 = tmp2 - K * H_mat(obst_idx);
            cov_mat = tmp2 * cov_mat;
            std::cout << "posterior cov = " << cov_mat << std::endl;
        }

        
        // arma::vec K = K_mat(0);                 // Equation (27)
    }



    std::mt19937 & get_random()
    {
        // static variables inside a function are created once and persist for the remainder of the program
        static std::random_device rd{}; 
        static std::mt19937 mt{rd()};
        // we return a reference to the pseudo-random number genrator object. This is always the
        // same object every time get_random is called
        return mt;
    }

    arma::vec gau_noise(double variance, unsigned n) {
        std::cout << "noise cov " << variance << std::endl;
        std::normal_distribution<> d(0, variance);

        arma::vec noises(n);
        for (unsigned i=0; i < n; i++) {
            noises(i) = d(get_random());
        }

        std::cout << "noises = " << noises << std::endl;
        return noises;
    }

}
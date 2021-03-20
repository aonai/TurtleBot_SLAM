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
        reset_cov();
    }

    StateVec::StateVec(arma::vec robot_state, arma::vec map_state, double state_noise_var, double sensor_noise_var):
        robot_state(robot_state), 
        map_state(map_state),
        state_noise_var(state_noise_var),
        sensor_noise_var(sensor_noise_var)
    {   
        robot_state(angle_idx) = rigid2d::normalize_angle(robot_state(angle_idx));
        reset_cov();
    }

    arma::mat StateVec::estimate(rigid2d::Twist2D t) {
        arma::mat A = A_mat(t);
        // arma::vec noises = gau_noise(state_noise_var, 3);
        arma::vec noises {0, 0, 0};

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
        return A;
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
        m(1) = rigid2d::normalize_angle(m(1));
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

        return H;
    }

    void StateVec::update_cov(arma::mat A) {
        unsigned n = 3 + map_state.size();
        arma::mat Q (n, n, arma::fill::zeros);
        for (unsigned r  = 0; r < 3; r++) {
            Q(r,r) = state_noise_var;
        }

        arma::mat new_cov_mat;
        new_cov_mat = A * cov_mat * A.t() + Q;
        cov_mat = new_cov_mat;
    }
    
    arma::mat StateVec::K_mat(arma::mat H) {
        arma::mat R (2,2);
        R(0,0) = sensor_noise_var;
        R(1,1) = sensor_noise_var;

        arma::mat K (3+map_state.size(), 2);
        K = H * cov_mat * H.t() + R;
        K = cov_mat * H.t() * arma::pinv(K, 0.01);

        return K;
    }

    void StateVec::ekf_update(rigid2d::Twist2D t, arma::vec actual_d, arma::vec actual_angle) {

        // check if there is new landmark, then re-initialize states and covariance 
        if (actual_d.size() > map_state.size()/2) {
            arma::vec new_map_state (actual_d.size() * 2); 
            for (unsigned j = 0; j < actual_d.size(); j++) { // Equation (23) and (24)
                if (actual_d[j] > 0) {
                    new_map_state(2*j) = robot_state(x_idx) + actual_d[j] * cos(actual_angle(j) + robot_state(angle_idx));
                    new_map_state(2*j+1) = robot_state(y_idx) + actual_d[j] * sin(actual_angle(j) + robot_state(angle_idx));
                }
                else {
                    new_map_state(2*j) = map_state(2*j);
                    new_map_state(2*j+1) = map_state(2*j+1);
                }
            }
            map_state = new_map_state;
            reset_cov();
            // std::cout << "update map = " << map_state << std::endl;
        }

        arma::mat A = estimate(t);
        update_cov(A);
        // std::cout << "update cov = " << cov_mat << std::endl;
        // std::cout << "update robot = " << robot_state << std::endl;

        for (unsigned obst_idx = 0; obst_idx < map_state.size()/2; obst_idx++) {
            if (actual_d(obst_idx) > 0) {
                arma::vec z = measurement_vec(obst_idx);       // Equation (25)

                arma::mat H = H_mat(obst_idx);
                // std::cout << "H = " << H;
                arma::mat K = K_mat(H);                        // Equation (26)
                // std::cout << "K = " << K;
                
                arma::vec z_act (2,arma::fill::zeros);
                z_act(0) = actual_d(obst_idx);
                z_act(1) = actual_angle(obst_idx);
                arma::vec z_diff = z_act - z;
                z_diff(1) = rigid2d::normalize_angle(z_diff(1));

                // std::cout << "actual z = " << z_act << std::endl;
                // std::cout << "estimate z = " << z << std::endl;
                // std::cout << "diff z = " << z_diff << std::endl;
                arma::vec tmp = K * z_diff;

                robot_state(0) = robot_state(0) + tmp(0);
                robot_state(1) = robot_state(1) + tmp(1);
                robot_state(2) = robot_state(2) + tmp(2);      // Equation (27)
                robot_state(angle_idx) = rigid2d::normalize_angle(robot_state(angle_idx));

                for (unsigned i =0 ; i < map_state.size(); i++) {
                    map_state(i) = map_state(i) + tmp(3+i);    // Equation (27)
                }
                // std::cout << "posterior robot = " << robot_state << std::endl;
                // std::cout << "posterior map = " << map_state << std::endl;

                arma::mat tmp2 (arma::size(cov_mat), arma::fill::eye);
                tmp2 = tmp2 - K * H;
                cov_mat = tmp2 * cov_mat;                      // Equation (28)
            }
        }

        // std::cout << "posterior robot = " << robot_state << std::endl;
        // std::cout << "posterior map = " << map_state << std::endl;
        // std::cout << "posterior cov = " << cov_mat << std::endl;
    }

    arma::vec StateVec::get_robot_state() {
        return robot_state;
    }

    arma::vec StateVec::get_map_state() {
        return map_state;
    }

    arma::mat StateVec::get_cov() {
        return cov_mat;
    }
    
    void StateVec::set_map_state(arma::vec ms) {
        map_state = ms;
        // std::cout << "set ms " << map_state.t() << std::endl;
        arma::mat last_cov = cov_mat;
        reset_cov();
        for (unsigned i = 0; i < last_cov.n_rows; i++) {
            for (unsigned j = 0; j < last_cov.n_cols; j++) {
                cov_mat(i, j) = last_cov(i, j);
            }
        }
        // std::cout << "new voc " << cov_mat << std::endl;
    }

    void StateVec::set_robot_state(arma::vec rs) {
        robot_state = rs;
    }


    void StateVec::reset_cov() {
        unsigned n_obst = map_state.size()/2;
        arma::mat map_cov (2*n_obst, 2*n_obst,  arma::fill::zeros);
        arma::mat robot_cov (3, 3,  arma::fill::zeros);

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
    }

    void StateVec::set_noises(double state_nv, double sensor_nv) {
        state_noise_var = state_nv;
        sensor_noise_var = sensor_nv;
    }

    std::mt19937 & get_random()
    {
        static std::random_device rd{}; 
        static std::mt19937 mt{rd()};
        return mt;
    }

    arma::vec gau_noise(double variance, unsigned n) {
        std::normal_distribution<> d(0, variance);

        arma::vec noises(n);
        for (unsigned i=0; i < n; i++) {
            noises(i) = d(get_random());
        }

        return noises;
    }

    int StateVec::check_association(arma::mat latest_cov) {
        // std::cout << "@asso ms = " << map_state.t() << std::endl;

        for (unsigned i = 0; i < latest_cov.n_rows; i++) {
            for (unsigned j = 0; j < latest_cov.n_cols; j++) {
                cov_mat(i, j) = latest_cov(i, j);
            }
        }

        // std::cout << "current cov = " << cov_mat << std::endl;

        arma::vec mah_dist(map_state.size()/2-1);
        for (unsigned k = 0; k < map_state.size()/2-1; k++) {
            // std::cout << "-------- start " << k << "---------------" << std::endl;
            arma::mat H = H_mat(k);
            // std::cout << "H = " << H << std::endl;

            arma::mat psi = H * cov_mat * H.t() + sensor_noise_var;
            // std::cout << "psi = " << psi << std::endl;

            arma::vec z_exp = measurement_vec(k); 
            z_exp(1) = rigid2d::normalize_angle(z_exp(1));
            // std::cout << "z_exp = " << z_exp.t() << std::endl;
            
            arma::vec z_measure = measurement_vec(map_state.size()/2-1); 
            z_measure(1) = rigid2d::normalize_angle(z_measure(1));
            // std::cout << "z_measure = " << z_measure.t() << std::endl;

            arma::vec z_diff = z_measure - z_exp;
            z_diff(1) = rigid2d::normalize_angle(z_diff(1));
            // std::cout << "z_diff = " << z_diff.t() << std::endl;
            
            arma::vec d = z_diff.t() * arma::pinv(psi, 1e-5) * z_diff;
            // std::cout << "d = " << d << std::endl;
            mah_dist(k) = d(0);

            // std::cout << "-------- end " << k << "---------------" << std::endl;
        }
        // std::cout << "mah_dist = " << mah_dist.t() << std::endl;
        
        double min_d = mah_dist.min();
        double min_idx = mah_dist.index_min();
        // std::cout << "min = " << min_d << " at " << min_idx << std::endl;

        if (min_d > 25) {
            // std::cout << "!!!!!!!!!!!!!!!!!! NEW" << std::endl;
            return map_state.size()/2-1;
        }
        else {
            return min_idx;
        }
        
        return -1;
    }

}
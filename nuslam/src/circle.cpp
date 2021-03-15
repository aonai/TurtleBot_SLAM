#include <iostream>
#include <cmath>
#include "nuslam/circle.hpp"

namespace circle {

    arma::field <arma::vec> group_clusters(arma::vec ranges, arma::vec angles){
        arma::field <arma::vec> clusters(1000);
        int count = 0;
        int group_num = 0;
        arma::vec group (ranges.size()*2);
        // std::cout << ranges.size() << " " << angles.size() << " " << group.size() << std::endl;
        
        //group clusters 
        for (unsigned i = 1; i < ranges.size(); i++) {
            group(2*count) = ranges(i-1);
            group(2*count+1) = angles(i-1);
            double dist = fabs(ranges(i)-ranges(i-1));
            // std::cout << "i = " << i << " " << dist << std::endl;

            if (dist > group_thresh || i == ranges.size()-1) {
                if (i == ranges.size()-1 && dist <= group_thresh) {
                    count += 1;
                    group(2*count) = ranges(i);
                    group(2*count+1) = angles(i);
                }
                group.resize(2*count+2);
                // std::cout << group_num << " group = " << group.t() << std::endl;
                clusters(group_num) = group;
                group_num += 1;
                count = 0;
                group = arma::vec (ranges.size()*2);
            }
            else {
                count += 1;
            }
        }

        //check first and last cluster 
        if (group_num > 1) {
            arma::vec first = clusters(0);
            arma::vec last = clusters(group_num-1);
            double dist = fabs(first(0) - last(last.size()-2));
            arma::vec tmp (first.size() + last.size());
            if (dist <= group_thresh) {
                for (unsigned i = 0; i < tmp.size(); i++) {
                    if (i < last.size()) {
                        tmp(i) = last(i);
                    }
                    else {
                        tmp(i) = first(i-last.size());
                    }
                }
                // std::cout << group_num << " tmp = " << tmp.t() << std::endl;
                group_num -= 1;
                clusters(0) = tmp;
            }
        }
        
        // remove small size clusters
        arma::field <arma::vec> clusters_tmp (group_num);
        int idx = 0;
        for (unsigned i = 0; i < group_num; i++) {
            if (clusters(i).size() >= 8) {
                clusters_tmp(idx) = clusters(i);
                idx += 1;
            }
        }
        // std::cout << "clusters_tmp = " << clusters_tmp << std::endl;
        // std::cout << "clusters_tmp = " << clusters_tmp.size() << " vs " << idx << std::endl;

        // resize clusters to return
        arma::field <arma::vec> toReturn (idx);
        for (unsigned i = 0; i < idx; i++) {
            toReturn(i) = clusters_tmp(i);
        }
        return toReturn;
    }

    Cluster::Cluster(arma::vec x, arma::vec y) : 
        x_arr(x),
        y_arr(y)
    {
        // std::cout << "x = " << x_arr.t() << std::endl;
        // std::cout << "y = " << y_arr.t() << std::endl;
    }

    Cluster::Cluster(arma::vec z) : z_arr(z) {
        // std::cout << "z = " << z_arr.t() << std::endl;
        x_arr = arma::vec (z_arr.size()/2);
        y_arr = arma::vec (z_arr.size()/2);
        ang_arr = arma::vec (z_arr.size()/2);

        for (unsigned i = 0; i < z_arr.size()/2; i ++) {
            double r = z_arr(2*i);
            double ang = z_arr(2*i+1);
            double x = r * cos(ang);
            double y = r * sin(ang); 
            x_arr(i) = x;
            y_arr(i) = y;
            ang_arr(i) = ang;
            // std::cout << ang*180/3.14 << " x y = " << x << " " << y << std::endl;
        }
    }

    Cluster::Cluster() {}
    
    void Cluster::fit_circle() {
        // std::cout << "angle = " << ang_arr.t() * 180/ 3.14 << std::endl;
        // std::cout << " x = " << x_arr.t() << std::endl;
        // std::cout << " y = " << y_arr.t() << std::endl;
        int n = x_arr.size();
        double x_mean = arma::mean(x_arr);
        double y_mean = arma::mean(y_arr);
        // std::cout << "mean = " << x_mean << " " << y_mean << std::endl;
        
        arma::vec x_tmp = x_arr - x_mean;
        arma::vec y_tmp = y_arr - y_mean;
        // std::cout << " x tmp = " << x_tmp.t() << std::endl;
        // std::cout << " y tmp = " << y_tmp.t() << std::endl;

        arma::vec z_tmp = pow(x_tmp,2) + pow(y_tmp,2);
        // std::cout << " z tmp = " << z_tmp.t() << std::endl;
        double z_mean = arma::mean(z_tmp);
        // std::cout << "z mean = " << z_mean << std::endl;

        arma::mat Z = join_rows(z_tmp, x_tmp);
        Z = join_rows(Z, y_tmp);
        arma::vec tmp (n, arma::fill::ones);
        Z = join_rows(Z, tmp);
        // std::cout << " Z = " << Z << std::endl;

        arma::mat M = Z.t() * Z / n;
        // std::cout << " M = " << M << std::endl;

        arma::mat H (4, 4, arma::fill::zeros);
        H(0, 0) = 8 * z_mean;
        H(0, 3) = 2;
        H(1, 1) = 1;
        H(2, 2) = 1;
        H(3, 0) = 2;
        arma::mat H_inv = arma::inv(H);
        // std::cout << " H = " << H << std::endl;
        // std::cout << " H inv= " << H_inv << std::endl;
        
        arma::mat U;
        arma::vec s;
        arma::mat V;
        arma::svd_econ(U,s,V,Z);
        // std::cout << " svd U = " << U.n_rows << " " << U.n_cols << std::endl;
        // std::cout << " svd s = " << s << std::endl;
        // std::cout << " svd V = " << V << std::endl;

        arma::mat sig (U.n_cols, V.n_rows, arma::fill::zeros);
        for (unsigned i = 0; i < V.n_rows; i++) {
            sig(i, i) = s(i);
        }
        // std::cout << " svd sig = " << sig << std::endl;

        arma::vec A;
        if (s.min() <= 1e-12) {
            A = V.col(3);
        }
        else {
            arma::mat Y = V * sig * V.t();
            // std::cout << "Y = " << Y << std::endl;
            
            arma::mat Q = Y * H_inv * Y;
            // std::cout << "Q = " << Q << std::endl;

            arma::cx_vec eigval;
            arma::cx_mat eigvec;

            arma::eig_gen(eigval, eigvec, Q);
            // std::cout << "eigval = " << eigval << std::endl;
            // std::cout << "eigvec = " << eigvec << std::endl;

            arma::vec eigval_tmp (eigval.size());
            arma::vec eigval_real = arma::real(eigval);
            for (unsigned i = 0; i < eigval.size(); i++) {
                if(eigval_real(i) >= 0) {
                    eigval_tmp(i) = eigval_real(i);
                }
                else {
                    eigval_tmp(i) = 1000;
                }
            }
            // std::cout << "eigval tmp = " << eigval_tmp << std::endl;
            int min_idx = eigval_tmp.index_min();
            // std::cout << "min_idx = " << min_idx << std::endl;

            arma::vec A_star = arma::real(eigvec.col(min_idx));
            // std::cout << "A_star = " << A_star << std::endl;

            A = (A_star.t() * arma::inv(Y)).t();
            // std::cout << "Y * A = " << Y * A << std::endl;

        }
        
        // std::cout << "A = " << A << std::endl;
        
        double a = -A(1) / (2 * A(0));
        double b = -A(2) / (2 * A(0));
        double R_sqr = (pow(A(1),2) + pow(A(2),2) - 4 * A(0) * A(3)) / (4 * pow(A(0),2));
        // std::cout << "a = " << a << std::endl;
        // std::cout << "b = " << b << std::endl;
        // std::cout << "R^2 = " << R_sqr << std::endl;

        // assign landmark center
        center_x = a + x_mean;
        center_y = b + y_mean;
        circle_radius = sqrt(R_sqr);
        // std::cout << "center = " << center_x << " " << center_y << std::endl;
        // std::cout << "R = " << sqrt(R_sqr) << std::endl;

        // check if landmark is circle
        arma::vec err_tmp = pow(x_tmp,2) + pow(y_tmp,2) - R_sqr;
        double rms_err = arma::accu(pow(err_tmp, 2));
        rms_err = sqrt(rms_err/n);
        // std::cout << "rms_err = " << rms_err << std::endl;
        
        if (rms_err <= is_circle_thresh) {
            is_circle = true;
        }
        else {
            is_circle = false;
        }
        // std::cout << "is_circle ?= " << is_circle << std::endl;
    }

    void Cluster::classify_arc() {
        if (is_circle == true) {
            double p1x = x_arr(0);
            double p1y = y_arr(0);
            double p2x = x_arr(x_arr.size()-1);
            double p2y = y_arr(y_arr.size()-1);
            arma::vec p_ang (x_arr.size());
            for (unsigned j = 0; j < x_arr.size(); j++) {
                double d1 = sqrt(pow(p1x-x_arr(j),2) + pow(p1y-y_arr(j),2));
                double d2 = sqrt(pow(p2x-x_arr(j),2) + pow(p2y-y_arr(j),2));
                p_ang(j) = atan2(d2, d1);
            }
            double ang_mean = arma::mean(p_ang) * 180/3.14159265358979323846;
            double ang_std = arma::stddev(p_ang);
            std::cout << " --- mean = " << ang_mean << " std_dev = " << ang_std << " R = " << circle_radius << std::endl;

            // if (ang_std > 0.5 || fabs(ang_mean) <= 90 || fabs(ang_mean) >= 135) {
            if (ang_std > 0.7) {
                is_circle = false;
            }

       
        }
    }
    
    bool Cluster::check_is_circle() {
        return is_circle;
    }

    double Cluster::get_center_x(){
        return center_x;
    }
    
    double Cluster::get_center_y(){
        return center_y;
    }

    double Cluster::get_circle_radius() {
        return circle_radius;
    }


}
